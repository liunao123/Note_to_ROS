import geopandas as gpd
from shapely.geometry import Point
from pyproj import CRS, Transformer
import os
import argparse
import yaml
from pathlib import Path


def load_config(config_file: str = None) -> dict:
    """
    加载配置文件
    
    Args:
        config_file: 配置文件路径，默认为脚本所在目录的 config.yaml
        
    Returns:
        配置字典
    """
    if config_file is None:
        script_dir = Path(__file__).parent
        config_file = script_dir / "config.yaml"
    
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    print(f"已加载配置文件: {config_path}")
    return config


def wgs84_to_utm(lon, lat):
    """
    将 WGS84 经纬度 (lon, lat) 转换到对应的 UTM 坐标
    自动判断 UTM 带号和南北半球
    """
    zone_number = int((lon + 180) / 6) + 1
    if lat >= 0:
        epsg_code = 32600 + zone_number  # 北半球
    else:
        epsg_code = 32700 + zone_number  # 南半球

    transformer = Transformer.from_crs(
        CRS.from_epsg(4326), CRS.from_epsg(epsg_code), always_xy=True
    )
    x, y = transformer.transform(lon, lat)
    return x, y, epsg_code


def clip_geojson(input_geojson_path, center_utm, radius_m, output_geojson_path):
    """
    根据 UTM 区域对输入 GeoJSON 处理
    - 点: 保留区域内的点
    - 线: 打断，保留区域范围内的部分
    - 面: 只要与区域相交，就保留整个 polygon
    """
    gdf = gpd.read_file(input_geojson_path)

    cx, cy = center_utm
    region = Point(cx, cy).buffer(radius_m)

    clipped_geoms = []
    for geom in gdf.geometry:
        if geom.is_empty:
            clipped_geoms.append(None)
            continue

        geom_type = geom.geom_type

        if geom_type in ["Point", "MultiPoint"]:
            if geom.within(region):
                clipped_geoms.append(geom)
            else:
                clipped_geoms.append(None)

        elif geom_type in ["LineString", "MultiLineString"]:
            inter = geom.intersection(region)
            if not inter.is_empty:
                clipped_geoms.append(inter)
            else:
                clipped_geoms.append(None)

        elif geom_type in ["Polygon", "MultiPolygon"]:
            if geom.intersects(region):
                clipped_geoms.append(geom)
            else:
                clipped_geoms.append(None)
        else:
            clipped_geoms.append(None)

    gdf["geometry"] = clipped_geoms
    gdf = gdf.dropna(subset=["geometry"])
    
    # 检查是否有有效数据
    if gdf.empty:
        print(f"⚠️  跳过 (区域内无数据): {os.path.basename(output_geojson_path)}")
        return False
    
    gdf.to_file(output_geojson_path, driver="GeoJSON")
    print(f"✅ 已输出 ({len(gdf)} 个要素): {os.path.basename(output_geojson_path)}")
    return True


def main():
    """主函数"""
    # 加载配置文件
    config = load_config()
    
    # 从配置文件读取参数
    # 从 search_params 读取中心点坐标和距离
    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    
    center_lat = default_location.get('latitude')
    center_lon = default_location.get('longitude')
    distance = search_params.get('distance', 10.0)  # 默认10米
    
    # 从 hd_map 读取输入输出文件夹配置（如果存在）
    hd_map_config = config.get('hd_map', {})
    input_folder = hd_map_config.get('input_folder')
    output_folder = hd_map_config.get('output_folder')
    
    # 如果 hd_map 中指定了 radius，使用它；否则使用 search_params 的 distance
    radius = hd_map_config.get('radius', distance)
    
    # 验证必需参数
    if center_lat is None or center_lon is None:
        raise ValueError("配置文件中缺少必需参数: search_params.default_location.latitude/longitude")
    if input_folder is None:
        raise ValueError("配置文件中缺少必需参数: hd_map.input_folder")
    if output_folder is None:
        raise ValueError("配置文件中缺少必需参数: hd_map.output_folder")
    
    print("="*80)
    print("高精地图裁切工具")
    print("="*80)
    print(f"输入文件夹: {input_folder}")
    print(f"输出文件夹: {output_folder}")
    print(f"中心点坐标: ({center_lat}, {center_lon})")
    print(f"裁切半径: {radius} 米")
    print("="*80)

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"✓ 已创建输出文件夹: {output_folder}")

    # 转换中心点坐标到UTM
    x, y, epsg_code = wgs84_to_utm(center_lon, center_lat)
    center_utm = (x, y)

    print(f"✓ 自动检测到 UTM 带号 EPSG:{epsg_code}")
    print(f"✓ 中心点 UTM 坐标: ({x:.2f}, {y:.2f})")
    print("="*80)

    # return

    # 获取所有 GeoJSON 文件
    geojson_files = [f for f in os.listdir(input_folder) if f.endswith(".geojson")]
    geojson_files.sort()
    
    if not geojson_files:
        print(f"⚠ 警告: 在 {input_folder} 中没有找到 .geojson 文件")
        return
    
    print(f"找到 {len(geojson_files)} 个 GeoJSON 文件")
    print("="*80)

    # 处理每个文件
    success_count = 0
    skip_count = 0
    
    for idx, file in enumerate(geojson_files, 1):
        print(f"[{idx}/{len(geojson_files)}] 处理: {file}")
        file_path = os.path.join(input_folder, file)
        output_file_path = os.path.join(output_folder, file)
        
        try:
            if clip_geojson(file_path, center_utm, radius, output_file_path):
                success_count += 1
            else:
                skip_count += 1
        except Exception as e:
            print(f"❌ 处理失败: {file} - {e}")
            skip_count += 1
    
    print("="*80)
    print(f"✓ 处理完成: {success_count} 个文件成功, {skip_count} 个文件跳过")
    print("="*80)


if __name__ == "__main__":
    main()
