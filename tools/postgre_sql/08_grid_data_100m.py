#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from glob import glob

import os
import sys
import argparse
import numpy as np
import yaml
import json
from pathlib import Path
from pypcd4 import PointCloud as PcdReader
import pyproj
# 使用qt.load_config的统一配置加载
from qt.load_config import load_config
from qt.box_projection import (
    get_all_session_dirs
)

def divide_line_by_lon_interval(lat1, lon1, lat2, lon2, interval_m=100):
    """
    给定两点经纬度，按经度方向每隔interval_m米等分，返回每个等分点的经纬度（包含起点和终点）。
    Args:
        lat1, lon1: 起点经纬度
        lat2, lon2: 终点经纬度
        interval_m: 经度方向等分距离，单位米
    Returns:
        List[(lat, lon)]: 每个等分点的经纬度
    """
    geod = pyproj.Geod(ellps='WGS84')
    # 只考虑经度方向
    # 计算总经度差
    total_dist, _ = geod.line_length([lon1, lon2], [lat1, lat2]), None
    # 计算总经度差（度）
    n_segments = 1
    # 计算在起点纬度下，经度每增加多少度是interval_m米
    lon_step = []
    lon_points = []
    # 先计算总距离
    az12, az21, total_dist = geod.inv(lon1, lat1, lon2, lat2)
    # 计算经度步长
    # 以lat1为纬度，向东移动interval_m，得到经度步长
    lon_east, _, _ = geod.fwd(lon1, lat1, 90, interval_m)
    lon_step_val = lon_east - lon1
    # 计算总步数
    n_segments = int(np.ceil(abs(lon2 - lon1) / abs(lon_step_val)))
    points = []
    for i in range(n_segments + 1):
        # 计算当前点经度
        if lon2 > lon1:
            lon = lon1 + i * abs(lon_step_val)
            if lon > lon2:
                lon = lon2
        else:
            lon = lon1 - i * abs(lon_step_val)
            if lon < lon2:
                lon = lon2
        # 计算该经度对应的纬度（线性插值）
        frac = (lon - lon1) / (lon2 - lon1) if lon2 != lon1 else 0
        lat = lat1 + frac * (lat2 - lat1)
        points.append((lat, lon))
        if lon == lon2:
            break
    return points
    
def write_points_to_kml(points, kml_path, name_prefix="Point"):
    """
    将一组经纬度点输出到KML文件，每个点一个Placemark。
    Args:
        points: List[(lat, lon)]
        kml_path: 输出KML文件路径
        name_prefix: Placemark名称前缀
    """
    with open(kml_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('  <Document>\n')
        for idx, (lat, lon) in enumerate(points):
            name = f"{name_prefix}_{idx+1}_{lat:.7f}_{lon:.7f}"
            f.write('    <Placemark>\n')
            f.write(f'      <name>{name}</name>\n')
            f.write('      <Style><IconStyle><color>ff0000ff</color><scale>1.0</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/blu-circle.png</href></Icon></IconStyle></Style>\n')
            f.write('      <Point>\n')
            f.write(f'        <coordinates>{lon},{lat},0</coordinates>\n')
            f.write('      </Point>\n')
            f.write('    </Placemark>\n')
        f.write('  </Document>\n')
        f.write('</kml>\n')
    print(f"已写入KML: {kml_path}")

def divide_line_by_distance(lat1, lon1, lat2, lon2, interval_m=100):
    """
    给定两点经纬度，将连线按interval_m（默认100m）等分，返回每个等分点的经纬度坐标（包含起点和终点）。
    Args:
        lat1, lon1: 起点经纬度
        lat2, lon2: 终点经纬度
        interval_m: 等分距离，单位米
    Returns:
        List[(lat, lon)]: 每个等分点的经纬度
    """
    geod = pyproj.Geod(ellps='WGS84')
    # 计算总距离和方位角
    az12, az21, total_dist = geod.inv(lon1, lat1, lon2, lat2)
    n_segments = int(np.ceil(total_dist / interval_m))
    points = []
    for i in range(n_segments + 1):
        dist = min(i * interval_m, total_dist)
        lon, lat, _ = geod.fwd(lon1, lat1, az12, dist)
        points.append((lat, lon))
    return points


def write_grid_kml(center_lat, center_lon, kml_path):
    """
    以指定点为中心，生成10x10个100m方格，覆盖1000x1000m区域，每个方格中心有经纬度Placemark。
    """

    geod = pyproj.Geod(ellps='WGS84')
    grid_size = 10
    cell_size = 100  # 单位：米
    # 以指定点为中心，向外延伸grid_size长度，生成每个格子的中心点
    half = grid_size // 2
    grid_centers = []
    for m in range(-half, grid_size - half):
        for n in range(-half, grid_size - half):
            # 以中心点为基准，m,n分别为南北和东西方向的偏移格数
            # 先南北方向
            lon1, lat1 = center_lon, center_lat
            lon2, lat2, _ = geod.fwd(lon1, lat1, 180, m * cell_size)
            # 再东西方向
            lon3, lat3, _ = geod.fwd(lon2, lat2, 90, n * cell_size)
            grid_centers.append((lat3, lon3, m, n))

    def get_square_coords(center_lat, center_lon, offset_m):
        # 四个方向
        n_lon, n_lat, _ = geod.fwd(center_lon, center_lat, 0, offset_m)
        s_lon, s_lat, _ = geod.fwd(center_lon, center_lat, 180, offset_m)
        e_lon, e_lat, _ = geod.fwd(center_lon, center_lat, 90, offset_m)
        w_lon, w_lat, _ = geod.fwd(center_lon, center_lat, 270, offset_m)
        # 四角
        nw_lon, nw_lat, _ = geod.fwd(w_lon, w_lat, 0, offset_m)
        ne_lon, ne_lat, _ = geod.fwd(e_lon, e_lat, 0, offset_m)
        se_lon, se_lat, _ = geod.fwd(e_lon, e_lat, 180, offset_m)
        sw_lon, sw_lat, _ = geod.fwd(w_lon, w_lat, 180, offset_m)
        coords = [
            (nw_lon, nw_lat),
            (ne_lon, ne_lat),
            (se_lon, se_lat),
            (sw_lon, sw_lat),
            (nw_lon, nw_lat)
        ]
        return coords

    with open(kml_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('  <Document>\n')
        f.write('    <Folder>\n')
        f.write('      <name>GridGroup</name>\n')
        for idx, (lat, lon, m, n) in enumerate(grid_centers):
            coords = get_square_coords(lat, lon, cell_size / 2)
            kml_coords = "\n".join([f"{x},{y},0" for x, y in coords])
            # 网格中心点名为m_n_lat_lon
            name = f"{m}_{n}_{lat:.7f}_{lon:.7f}"
            f.write('      <Placemark>\n')
            f.write(f'        <name>{name}</name>\n')
            f.write('        <Style><LineStyle><color>ff00ff00</color><width>2</width></LineStyle><PolyStyle><color>3300ff00</color></PolyStyle></Style>\n')
            f.write('        <Polygon>\n')
            f.write('          <outerBoundaryIs>\n')
            f.write('            <LinearRing>\n')
            f.write('              <coordinates>\n')
            f.write(kml_coords + '\n')
            f.write('              </coordinates>\n')
            f.write('            </LinearRing>\n')
            f.write('          </outerBoundaryIs>\n')
            f.write('        </Polygon>\n')
            f.write('      </Placemark>\n')
            # 每个方格中心点Placemark
            f.write('      <Placemark>\n')
            f.write(f'        <name>{name}</name>\n')
            f.write('        <Style><IconStyle><color>ff0000ff</color><scale>0.8</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/blu-circle.png</href></Icon></IconStyle></Style>\n')
            f.write('        <Point>\n')
            f.write(f'          <coordinates>{lon},{lat},0</coordinates>\n')
            f.write('        </Point>\n')
            f.write('      </Placemark>\n')
        f.write('    </Folder>\n')
        f.write('  </Document>\n')
        f.write('</kml>\n')
    print(f"已写入KML: {kml_path}")

    
def write_centered_rectangles_kml(center_lat, center_lon, kml_path):
    """
    以指定点为基准，沿N/S/E/W四个方向，每隔100m生成一个100m×100m的矩形，总共每个方向10个（中心±500m），并在中心点添加Placemark。
    所有内容写入同一个KML文件。
    """
    import pyproj
    geod = pyproj.Geod(ellps='WGS84')

    def get_square_coords(center_lat, center_lon, offset_m):
        # 四个方向
        n_lon, n_lat, _ = geod.fwd(center_lon, center_lat, 0, offset_m)
        s_lon, s_lat, _ = geod.fwd(center_lon, center_lat, 180, offset_m)
        e_lon, e_lat, _ = geod.fwd(center_lon, center_lat, 90, offset_m)
        w_lon, w_lat, _ = geod.fwd(center_lon, center_lat, 270, offset_m)
        # 四角
        nw_lon, nw_lat, _ = geod.fwd(w_lon, w_lat, 0, offset_m)
        ne_lon, ne_lat, _ = geod.fwd(e_lon, e_lat, 0, offset_m)
        se_lon, se_lat, _ = geod.fwd(e_lon, e_lat, 180, offset_m)
        sw_lon, sw_lat, _ = geod.fwd(w_lon, w_lat, 180, offset_m)
        coords = [
            (nw_lon, nw_lat),
            (ne_lon, ne_lat),
            (se_lon, se_lat),
            (sw_lon, sw_lat),
            (nw_lon, nw_lat)
        ]
        return coords

    # 生成所有矩形的中心点（包括中心点本身）
    rect_centers = [(center_lat, center_lon, 'Center')]
    for i in range(1, 6):
        # 北
        n_lon, n_lat, _ = geod.fwd(center_lon, center_lat, 0, i * 100)
        rect_centers.append((n_lat, n_lon, f'N+{i*100}m'))
        # 南
        s_lon, s_lat, _ = geod.fwd(center_lon, center_lat, 180, i * 100)
        rect_centers.append((s_lat, s_lon, f'S+{i*100}m'))
        # 东
        e_lon, e_lat, _ = geod.fwd(center_lon, center_lat, 90, i * 100)
        rect_centers.append((e_lat, e_lon, f'E+{i*100}m'))
        # 西
        w_lon, w_lat, _ = geod.fwd(center_lon, center_lat, 270, i * 100)
        rect_centers.append((w_lat, w_lon, f'W+{i*100}m'))

    # 写KML
    with open(kml_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('  <Document>\n')
        # 画所有矩形
        for lat, lon, label in rect_centers:
            coords = get_square_coords(lat, lon, 50)
            kml_coords = "\n".join([f"{x},{y},0" for x, y in coords])
            name = f"Square_100m_{lat:.7f}_{lon:.7f}_{label}"
            f.write('    <Placemark>\n')
            f.write(f'      <name>{name}</name>\n')
            f.write('      <Style><LineStyle><color>ff00ff00</color><width>2</width></LineStyle><PolyStyle><color>3300ff00</color></PolyStyle></Style>\n')
            f.write('      <Polygon>\n')
            f.write('        <outerBoundaryIs>\n')
            f.write('          <LinearRing>\n')
            f.write('            <coordinates>\n')
            f.write(kml_coords + '\n')
            f.write('            </coordinates>\n')
            f.write('          </LinearRing>\n')
            f.write('        </outerBoundaryIs>\n')
            f.write('      </Polygon>\n')
            f.write('    </Placemark>\n')
        # 中心点Placemark
        f.write('    <Placemark>\n')
        f.write(f'      <name>Center: {center_lat:.7f}, {center_lon:.7f}</name>\n')
        f.write('      <Style><IconStyle><color>ff0000ff</color><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/blu-circle.png</href></Icon></IconStyle></Style>\n')
        f.write('      <Point>\n')
        f.write(f'        <coordinates>{center_lon},{center_lat},0</coordinates>\n')
        f.write('      </Point>\n')
        f.write('    </Placemark>\n')
        f.write('  </Document>\n')
        f.write('</kml>\n')
    print(f"已写入KML: {kml_path}")





def load_3dbox_from_json(json_file):
    """
    从JSON文件中加载3D box标注数据
    
    Args:
        json_file: JSON文件路径
        
    Returns:
        list: 包含所有3D box的列表，每个box是一个字典，包含以下字段：
            - 'position': dict with 'x', 'y', 'z'
            - 'rotation': dict with 'x', 'y', 'z' (欧拉角)
            - 'size': dict with 'x', 'y', 'z'
            - 'label_name': 标签名称
            - 'alias_name': 别名
            - 'uuid': 唯一标识符
            - 'track_id': 跟踪ID
            返回空列表如果文件不存在或解析失败
    """
    if not os.path.exists(json_file):
        # print(f"Warning: Label file not found: {json_file}")
        return []
    
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        boxes = []
        
        # 检查是否有 'objects' 字段
        if 'objects' not in data or not isinstance(data['objects'], list):
            # print(f"Warning: No 'objects' array found in {json_file}")
            return []
        
        # 遍历所有对象
        for obj in data['objects']:
            if not isinstance(obj, dict):
                continue
            
            # 提取几何信息
            geometry = obj.get('geometry', {})
            if not geometry:
                continue
            
            box = {
                'position': geometry.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
                'rotation': geometry.get('rotation', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
                'size': geometry.get('size', {'x': 0.0, 'y': 0.0, 'z': 0.0}),
                'label_name': obj.get('labelName', ''),
                'alias_name': obj.get('aliasName', ''),
                'uuid': obj.get('uuid', ''),
                'track_id': obj.get('trackId', -1),
                'tool_type': obj.get('toolType', ''),
                'color': obj.get('color', ''),
                'is_dynamic': obj.get('isDynamic', False)
            }
            
            boxes.append(box)
        
        return boxes
        
    except json.JSONDecodeError as e:
        print(f"Error: Failed to parse JSON file {json_file}: {e}")
        return []
    except Exception as e:
        print(f"Error: Failed to load 3D boxes from {json_file}: {e}")
        return []


def calculate_utm_zone(longitude: float) -> int:
    """
    根据经度自动计算UTM区号
    
    Args:
        longitude: 经度 (-180 到 180)
        
    Returns:
        UTM区号 (1-60)
    """
    zone = int((longitude + 180) / 6) + 1
    return max(1, min(60, zone))


def calculate_utm_epsg(longitude: float, latitude: float = 0) -> int:
    """
    根据经纬度自动计算UTM EPSG代码
    
    Args:
        longitude: 经度 (-180 到 180)
        latitude: 纬度 (-90 到 90)，用于判断南北半球
        
    Returns:
        EPSG代码 (326XX for 北半球, 327XX for 南半球)
    """
    zone = calculate_utm_zone(longitude)
    # 北半球: EPSG:326XX, 南半球: EPSG:327XX
    if latitude >= 0:
        return 32600 + zone
    else:
        return 32700 + zone


def write_las_file(filename, points, intensities=None, epsg_code=32651 ):
    """
    保存点云为LAS文件格式
    
    参数:
        filename (str): 输出LAS文件路径
        points (np.ndarray): 点云坐标数组 (N, 3)
        intensities (np.ndarray): 强度数组 (N,)
        epsg_code (int): EPSG代码，默认32651 (WGS 84 / UTM zone 51N)
        offset_utm (np.ndarray): UTM偏移量 [x, y, z]，用于设置LAS header offset
    
    返回:
        bool: 成功返回True，失败返回False
    """
    try:
        # 创建LAS header (LAS 1.2, Point Format 1 支持强度)
        header = laspy.LasHeader(point_format=1, version="1.2")
        
        # 设置投影信息
        crs = pyproj.CRS.from_epsg(epsg_code)
        header.add_crs(crs)
        
        # 设置偏移量和缩放因子
        # 使用点云的最小值作为偏移量（向下取整到米）
        min_x = np.floor(points[:, 0].min())
        min_y = np.floor(points[:, 1].min())
        min_z = np.floor(points[:, 2].min())
        
        header.offsets = [min_x, min_y, min_z]
        header.scales = [0.001, 0.001, 0.001]  # 1mm精度
        
        # print(f"  点云范围:")
        # print(f"    X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
        # print(f"    Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
        # print(f"    Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
        # print(f"  LAS偏移量: [{min_x}, {min_y}, {min_z}]")
        
        # 创建LAS数据对象
        las = laspy.LasData(header)
        
        # 设置坐标
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]
        
        # 设置强度 (LAS 强度字段为 uint16：0-65535)
        if intensities is not None:
            # 旧逻辑（已弃用）：把强度当作 0-255 并映射到 0-65535。
            # intensity_normalized = np.clip(intensities, 0, 255)
            # las.intensity = (intensity_normalized * 257).astype(np.uint16)

            # 按需求：强度值“直接写入”，不做任何缩放/归一化/裁剪。
            # 注意：LAS 只能存 uint16。如果原始强度不在 [0, 65535]，直接转 uint16 会发生回绕，
            # 因此这里做严格校验，超范围直接报错。
            min_i = float(np.min(intensities))
            max_i = float(np.max(intensities))
            if min_i < 0.0 or max_i > 65535.0:
                raise ValueError(
                    f"intensity out of uint16 range [0,65535]: min={min_i}, max={max_i}. "
                    "请先在上游保证强度范围，或改用裁剪/缩放策略。"
                )
            las.intensity = intensities.astype(np.uint16)
        else:
            las.intensity = np.zeros(len(points), dtype=np.uint16)
        
        # 保存LAS文件
        las.write(filename)
        
        # print(f"  偏移量: {header.offsets}")
        
        return True
        
    except Exception as e:
        print(f"保存LAS文件失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def extract_timestamp(filename):
    """从文件名中提取时间戳"""
    try:
        parts = filename.split('_')
        if len(parts) >= 2:
            timestamp = parts[1].split('.')[0]
            return timestamp
    except:
        pass
    return None


def read_pose_from_yaml(yaml_file):
    """从YAML文件中读取位姿数据"""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    # 读取时间戳
    timestamp = data.get('timestamp', 0.0)
    
    # 读取 pose_utm (4x4变换矩阵，存储为16元素列表)
    pose_utm = data.get('pose_utm', None)
    if pose_utm is None:
        raise ValueError(f"pose_utm not found in {yaml_file}")
    
    # 将16元素列表转换为4x4矩阵
    if isinstance(pose_utm, list) and len(pose_utm) == 16:
        transformation_matrix = np.array(pose_utm, dtype=np.float64).reshape(4, 4)
    else:
        raise ValueError(f"Invalid pose_utm format in {yaml_file}")
    
    # 读取UTM偏移量（列表格式: [x, y, z]）
    offset_utm = data.get('offset_utm', [0.0, 0.0, 0.0])
    if isinstance(offset_utm, list) and len(offset_utm) >= 3:
        utm_offset = np.array([offset_utm[0], offset_utm[1], offset_utm[2]], dtype=np.float64)
    else:
        utm_offset = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    
    return {
        'transformation_matrix': transformation_matrix,
        'offset_utm': utm_offset,
        'timestamp': timestamp
    }


def read_pcd_file(pcd_path):
    """
    读取PCD文件，返回点云数据
    使用 pypcd4 库读取（支持压缩格式和强度信息）
    """
    try:
        pcd_data = PcdReader.from_path(pcd_path)
        points = pcd_data.numpy(("x", "y", "z", "intensity"))
        
        # 分离点坐标和强度
        xyz = points[:, :3].astype(np.float32)
        intensities = points[:, 3].astype(np.float32)
        
        return {'points': xyz, 'intensities': intensities}
    except Exception as e:
        print(f"Error reading PCD file {pcd_path}: {e}")
        raise


def filter_pointcloud(points, intensities=None, min_distance=3.0, min_z=-4.0, max_z=20.0):
    """
    过滤点云：
    1. 移除距离原点小于min_distance的点
    2. 移除Z坐标超出范围的点
    """
    # 计算到原点的距离
    distances = np.linalg.norm(points, axis=1)
    
    # 创建过滤掩码
    mask = (distances >= min_distance) & (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    
    # 应用过滤
    filtered_points = points[mask]
    
    if intensities is not None:
        filtered_intensities = intensities[mask]
    else:
        filtered_intensities = np.zeros(len(filtered_points), dtype=np.float32)
    
    return filtered_points, filtered_intensities


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    将欧拉角（ZYX顺序）转换为旋转矩阵
    
    Args:
        roll: 绕X轴旋转角度（弧度）
        pitch: 绕Y轴旋转角度（弧度）
        yaw: 绕Z轴旋转角度（弧度）
        
    Returns:
        3x3 旋转矩阵
    """
    # 绕Z轴旋转（yaw）
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    Rz = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    # 绕Y轴旋转（pitch）
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    Ry = np.array([
        [cos_pitch, 0, sin_pitch],
        [0, 1, 0],
        [-sin_pitch, 0, cos_pitch]
    ])
    
    # 绕X轴旋转（roll）
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    Rx = np.array([
        [1, 0, 0],
        [0, cos_roll, -sin_roll],
        [0, sin_roll, cos_roll]
    ])
    
    # 组合旋转矩阵 R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R


def point_in_box(points, box):
    """
    判断点是否在3D box内
    
    Args:
        points: 点云数组 (N, 3)
        box: 3D box字典，包含 position, rotation, size
        
    Returns:
        mask: 布尔数组 (N,)，True表示点在box内
    """
    # 提取box参数
    center = np.array([box['position']['x'], 
                       box['position']['y'], 
                       box['position']['z']], dtype=np.float32)
    
    # 欧拉角（假设是弧度，如果是角度需要转换）
    rotation = box['rotation']
    roll = rotation['x']
    pitch = rotation['y']
    yaw = rotation['z']
    
    # box尺寸
    size = np.array([box['size']['x'], 
                     box['size']['y'], 
                     box['size']['z']], dtype=np.float32)
    
    # 半尺寸
    half_size = size / 2.0
    
    # 计算旋转矩阵
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    
    # 将点转换到box的局部坐标系
    # 1. 平移到box中心
    points_centered = points - center
    
    # 2. 旋转到box的局部坐标系（使用旋转矩阵的转置）
    points_local = (R.T @ points_centered.T).T
    
    # 3. 检查点是否在box的边界内
    mask = (
        (np.abs(points_local[:, 0]) <= half_size[0]) &
        (np.abs(points_local[:, 1]) <= half_size[1]) &
        (np.abs(points_local[:, 2]) <= half_size[2])
    )
    
    return mask


def remove_points_within_3dbox(points, intensities, boxes, remove_dynamic_only=False):
    """
    删除落在3D box内的点
    
    Args:
        points: 点云数组 (N, 3)，会被就地修改
        intensities: 强度数组 (N,)，会被就地修改
        boxes: 3D box列表
        
    Returns:
        filtered_points: 过滤后的点云 (M, 3)
        filtered_intensities: 过滤后的强度 (M,)
    """
    if len(boxes) == 0:
        return points, intensities
    
    # 创建掩码，初始值为True（保留所有点）
    keep_mask = np.ones(len(points), dtype=bool)
    
    # 对每个box，标记其内部的点
    removed_count = 0
    for box in boxes:
        if remove_dynamic_only:
            # 只处理动态物体
            if not box.get('is_dynamic', False):
                continue
 
        # 检查点是否在当前box内
        in_box_mask = point_in_box(points, box)
        
        # 更新保留掩码（在box内的点设为False）
        keep_mask &= ~in_box_mask
        
        box_removed = np.sum(in_box_mask)
        if box_removed > 0:
            removed_count += box_removed
    
    # 应用掩码过滤点云
    filtered_points = points[keep_mask]
    filtered_intensities = intensities[keep_mask]
    
    if removed_count > 0:
        original_count = len(points)
        filtered_count = len(filtered_points)
        # print(f"    移除了 {removed_count} 个点 (在{len(boxes)}个box内), "
        #       f"剩余 {filtered_count}/{original_count} 个点")
    
    return filtered_points, filtered_intensities


def merge_session_pointclouds(pointclouds_dir, labels_dir, poses_dir, resample=1, remove_dynamic_only=False):
    """
    合并一个session中的所有点云到统一坐标系
    
    Args:
        pointclouds_dir: 点云文件目录
        labels_dir: 标签文件目录
        poses_dir: 位姿文件目录
        resample: 采样率，1表示使用所有点云
        remove_dynamic_only: 是否只移除动态物体，True表示只移除动态物体，False表示移除所有3D box内的物体
        
    Returns:
        dict: 包含合并结果的字典
            - 'points': np.ndarray, 相对坐标点云 (N, 3)
            - 'intensities': np.ndarray, 强度值 (N,)
            - 'first_pose': np.ndarray, UTM偏移量 [x, y, z]
            - 'processed_count': int, 成功处理的文件数
            - 'error_count': int, 失败的文件数
    """
    print("=" * 60)
    print("Merge Session Pointclouds")
    print("=" * 60)
    print(f"Pointclouds directory: {pointclouds_dir}")
    print(f"Poses directory: {poses_dir}")
    print(f"Resample rate: {resample}")
    print("=" * 60)
    
    # 获取所有PCD文件
    pcd_files = sorted([f for f in os.listdir(pointclouds_dir) if f.endswith('.pcd')])
    pcd_files = sorted(pcd_files)

    print(f"Found {len(pcd_files)} PCD files")
    
    if len(pcd_files) == 0:
        print("Error: No PCD files found!")
        return
    
    # 创建合并后的点云列表
    merged_points = []
    merged_intensities = []
    
    processed_count = 0
    error_count = 0
    
    # Hard-coded first pose (手动指定第一个位姿的UTM偏移量)
    # 如果需要使用固定的参考点，取消注释下面这行并设置具体数值
    # first_pose = np.array([your_x, your_y, your_z], dtype=np.float64)
    first_pose = None  # 设置为None则自动使用第一个文件的offset_utm

    # 采样率，1表示使用所有点云文件，2表示每隔一个文件使用一个，依此类推
    
    for idx, pcd_file in enumerate(pcd_files):

        if idx % resample != 0:
            continue

        try:
            # 提取时间戳
            timestamp_str = extract_timestamp(pcd_file)
            if timestamp_str is None:
                print(f"Warning: Could not extract timestamp from {pcd_file}")
                error_count += 1
                continue
            
            # 构建对应的YAML文件名
            yaml_file = os.path.join(poses_dir, pcd_file.replace('.pcd', '.yaml'))
            label_file = os.path.join(labels_dir, pcd_file.replace('.pcd', '.json'))

            # 检查YAML文件是否存在
            if not os.path.exists(yaml_file):
                print(f"Warning: Pose file not found: {yaml_file}")
                error_count += 1
                continue
            
            # 读取点云
            pcd_path = os.path.join(pointclouds_dir, pcd_file)
            # print(f"Processing file {idx+1}/{len(pcd_files)}: {pcd_path} {yaml_file}")
            if (idx+1) % 50 == 0 or idx == 0:
                print(f"Processing file   {idx+1} / {len(pcd_files)} ......")

            cloud_data = read_pcd_file(pcd_path)
            points = cloud_data['points']
            intensities = cloud_data['intensities']
            
            if len(points) == 0:
                print(f"Warning: Empty point cloud: {pcd_path}")
                error_count += 1
                continue

            # 过滤点云
            filtered_points, filtered_intensities = filter_pointcloud(
                points, intensities, min_distance=3.0, min_z=-4.0, max_z=20.0
            )
            
            if len(filtered_points) == 0:
                print(f"Warning: All points filtered out from {pcd_file}")
                error_count += 1
                continue

            # 读取3D box标签并移除对应点
            # out_boxs = load_3dbox_from_json(label_file)
            # if len(out_boxs) > 0:
            #     # print(f"  Loaded {len(out_boxs)} boxes from {label_file}")
            #     filtered_points, filtered_intensities = remove_points_within_3dbox(
            #         filtered_points, filtered_intensities, out_boxs, remove_dynamic_only=remove_dynamic_only
            #     )
            
            # 读取位姿数据
            pose_data = read_pose_from_yaml(yaml_file)
            
            # 记录第一个位姿作为参考
            if first_pose is None:
                first_pose = pose_data['offset_utm']
                print(f"First pose (UTM offset): {first_pose}")
            
            # 应用变换矩阵
            # 将点云转换为齐次坐标
            ones = np.ones((len(filtered_points), 1), dtype=np.float32)
            homogeneous_points = np.hstack([filtered_points, ones])
            
            # 应用4x4变换矩阵
            transformed_points = (pose_data['transformation_matrix'] @ homogeneous_points.T).T
            
            # 提取前3列（x, y, z）
            transformed_points = transformed_points[:, :3]
            
            # 添加UTM偏移量
            transformed_points += pose_data['offset_utm'] - first_pose
            
            # 添加到合并列表
            merged_points.append(transformed_points)
            merged_intensities.append(filtered_intensities)
            
            processed_count += 1
            
            # 每处理100个文件输出一次进度
            if processed_count % 100 == 0:
                total_points = sum(len(p) for p in merged_points)
                print(f"Processed {processed_count}/{len(pcd_files)} files... "
                      f"Total points: {total_points}")
        
        except Exception as e:
            print(f"Error processing {pcd_file}: {str(e)}")
            import traceback
            traceback.print_exc()
            error_count += 1
            continue
    
    print("=" * 60)
    print(f"Processing completed!")
    print(f"Successfully processed: {processed_count} files")
    print(f"Errors: {error_count} files")
    
    if len(merged_points) == 0:
        print("Error: No points in merged cloud!")
        return None
    
    # 合并所有点云
    all_points = np.vstack(merged_points).astype(np.float32)
    all_intensities = np.hstack(merged_intensities).astype(np.float32)
    
    print(f"Total merged points: {len(all_points)}")
    print(f"Local UTM offset: {first_pose}")
    
    # 打印点云统计信息
    print("=" * 60)
    print("Point Cloud Statistics:")
    print(f"X range: [{all_points[:, 0].min():.3f}, {all_points[:, 0].max():.3f}] m")
    print(f"Y range: [{all_points[:, 1].min():.3f}, {all_points[:, 1].max():.3f}] m")
    print(f"Z range: [{all_points[:, 2].min():.3f}, {all_points[:, 2].max():.3f}] m")
    print("=" * 60)
    
    # 返回合并结果
    return {
        'points': all_points,
        'intensities': all_intensities,
        'first_pose': first_pose,
        'processed_count': processed_count,
        'error_count': error_count
    }


def filter_pointcloud_by_location(points, intensities, first_pose, center_lon, center_lat, distance, utm_zone):
    """
    根据GPS位置过滤点云，只保留指定距离范围内的点
    
    Args:
        points: 相对坐标点云 (N, 3)
        intensities: 强度值 (N,)
        first_pose: UTM偏移量 [x, y, z]
        center_lon: 中心点经度
        center_lat: 中心点纬度
        distance: 距离范围（米），将创建 2*distance x 2*distance 的矩形框
        utm_zone: UTM区号
        
    Returns:
        filtered_points: 过滤后的点云 (M, 3)
        filtered_intensities: 过滤后的强度值 (M,)
    """
    import pyproj
    
    # 将相对坐标转换为绝对UTM坐标
    absolute_points = points + first_pose
    
    # 将目标GPS坐标转换为UTM坐标
    utm_srid = 32600 + utm_zone  # 北半球
    wgs84 = pyproj.CRS('EPSG:4326')
    utm = pyproj.CRS(f'EPSG:{utm_srid}')
    transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)
    
    center_utm_x, center_utm_y = transformer.transform(center_lon, center_lat)
    
    print(f"\n空间过滤:")
    print(f"  中心点GPS: ({center_lat:.6f}, {center_lon:.6f})")
    print(f"  中心点UTM: ({center_utm_x:.3f}, {center_utm_y:.3f})")
    print(f"  过滤范围: ±{distance:.1f}m (矩形框: {2*distance:.1f}m x {2*distance:.1f}m)")
    
    # 计算矩形框边界
    min_x = center_utm_x - distance
    max_x = center_utm_x + distance
    min_y = center_utm_y - distance
    max_y = center_utm_y + distance
    
    print(f"  X范围: [{min_x:.3f}, {max_x:.3f}]")
    print(f"  Y范围: [{min_y:.3f}, {max_y:.3f}]")
    
    # 创建过滤掩码（矩形框）
    mask = (
        (absolute_points[:, 0] >= min_x) & 
        (absolute_points[:, 0] <= max_x) &
        (absolute_points[:, 1] >= min_y) & 
        (absolute_points[:, 1] <= max_y)
    )
    
    # 应用过滤
    filtered_absolute_points = absolute_points[mask]
    filtered_intensities = intensities[mask]
    
    # 转换回相对坐标
    filtered_points = filtered_absolute_points - first_pose
    
    original_count = len(points)
    filtered_count = len(filtered_points)
    removed_count = original_count - filtered_count
    
    print(f"  原始点数: {original_count:,}")
    print(f"  保留点数: {filtered_count:,}")
    print(f"  移除点数: {removed_count:,} ({removed_count/original_count*100:.1f}%)")
    
    return filtered_points, filtered_intensities

 



def load_and_fit_pose_line(poses_dir):
    """
    加载poses_dir下所有pose，拟合一条直线并可视化
    Args:
        poses_dir: pose yaml文件所在目录
    Returns:
        poses_xyz: 所有pose的xyz坐标 (N,3)
        line_params: 拟合直线参数
    """
    pose_files = sorted(glob(os.path.join(poses_dir, '*.yaml')))
    if not pose_files:
        print(f"[ERROR] No pose yaml files found in {poses_dir}")
        return None, None
    poses = []
    for pf in pose_files:
        try:
            pose_data = read_pose_from_yaml(pf)
            # 取transformation_matrix的平移部分 (最后一列的前3个元素)
            T = pose_data['transformation_matrix']
            translation = T[:3, 3]
            # print(f"Loaded pose from {pf}: translation = {translation}")
            poses.append(translation)
        except Exception as e:
            print(f"[WARN] Failed to read {pf}: {e}")
    poses_xyz = np.array(poses)
    if poses_xyz.shape[0] < 2:
        print("[WARN] Too few poses for line fitting.")
        return poses_xyz, None
    # 只用xy平面做直线拟合
    x = poses_xyz[:,0]
    y = poses_xyz[:,1]
    # 拟合y = kx + b
    k, b = np.polyfit(x, y, 1)
    print(f"拟合直线: y = {k:.6f} * x + {b:.6f}")
    # 可视化
    plt.figure(figsize=(8,6))
    plt.plot(x, y, 'o', label='poses')
    x_fit = np.linspace(x.min(), x.max(), 100)
    y_fit = k * x_fit + b
    plt.plot(x_fit, y_fit, 'r-', label='fitted line')
    plt.xlabel('X (UTM)')
    plt.ylabel('Y (UTM)')
    plt.title('Pose Trajectory and Fitted Line')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    img_path = os.path.join(poses_dir, 'pose_fit.png')
    plt.savefig(img_path, dpi=150, bbox_inches='tight')
    print(f"可视化已保存: {img_path}")
    plt.close()
    return poses_xyz, (k, b)

def write_rectangle_kml(center_lat, center_lon, offset_m, kml_path):
    import pyproj
    geod = pyproj.Geod(ellps='WGS84')
    # 计算四个顶点（顺时针：北、西、南、东）
    # 北
    n_lon, n_lat, _ = geod.fwd(center_lon, center_lat, 0, offset_m)
    # 南
    s_lon, s_lat, _ = geod.fwd(center_lon, center_lat, 180, offset_m)
    # 东
    e_lon, e_lat, _ = geod.fwd(center_lon, center_lat, 90, offset_m)
    # 西
    w_lon, w_lat, _ = geod.fwd(center_lon, center_lat, 270, offset_m)
    # 四个角点（顺时针：NW, NE, SE, SW, 再回到NW闭合）
    # 先求四角坐标
    # NW: 北+西
    nw_lon, nw_lat, _ = geod.fwd(w_lon, w_lat, 0, offset_m)
    # NE: 北+东
    ne_lon, ne_lat, _ = geod.fwd(e_lon, e_lat, 0, offset_m)
    # SE: 南+东
    se_lon, se_lat, _ = geod.fwd(e_lon, e_lat, 180, offset_m)
    # SW: 南+西
    sw_lon, sw_lat, _ = geod.fwd(w_lon, w_lat, 180, offset_m)
    # 组织成KML坐标字符串（闭合）
    coords = [
        (nw_lon, nw_lat),
        (ne_lon, ne_lat),
        (se_lon, se_lat),
        (sw_lon, sw_lat),
        (nw_lon, nw_lat)
    ]
    kml_coords = "\n".join([f"{lon},{lat},0" for lon, lat in coords])
    with open(kml_path, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('  <Document>\n')
        f.write('    <Placemark>\n')
        placemark_name = f"Rectangle_{offset_m}m_{center_lat:.7f}_{center_lon:.7f}"
        f.write(f'      <name>{placemark_name}</name>\n')
        f.write('      <Style><LineStyle><color>ff0000ff</color><width>3</width></LineStyle><PolyStyle><color>330000ff</color></PolyStyle></Style>\n')
        f.write('      <Polygon>\n')
        f.write('        <outerBoundaryIs>\n')
        f.write('          <LinearRing>\n')
        f.write('            <coordinates>\n')
        f.write(kml_coords + '\n')
        f.write('            </coordinates>\n')
        f.write('          </LinearRing>\n')
        f.write('        </outerBoundaryIs>\n')
        f.write('      </Polygon>\n')
        f.write('    </Placemark>\n')
        f.write('  </Document>\n')
        f.write('</kml>\n')
    print(f"已写入KML: {kml_path}")


def main():
    config = load_config()
    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    # 搜索参数 复用 search_params 中的默认位置
    target_lat = default_location.get('latitude', 31.41033324)
    target_lon = default_location.get('longitude', 120.65582103)
    print(f"中心点坐标: ({target_lat}, {target_lon})")

    # 调用函数，生成10x10方格KML
    write_grid_kml(target_lat, target_lon, "/data/exported_roi_data/temp/grid_10x10.kml")

    lat1, lon1 = 31.4245088, 120.6266182
    lat2, lon2 = 31.4263288, 120.6193545

    pts = divide_line_by_lon_interval(lat1, lon1, lat2, lon2, 100)
    print("Line division points (every 100m in longitude direction):")
    for lat, lon in pts:
        print(f"  ({lat:.7f}, {lon:.7f})")
    # 输出到KML
    write_points_to_kml(pts, "/data/line_division_points.kml", name_prefix="DivPt")
    
    exit(0)

    # 获取所有session目录
    session_data_dir = '/data/dwm_data/'
    session_dirs = get_all_session_dirs(session_data_dir)
    total_sessions = len(session_dirs)
    print(f'Found {len(session_dirs)} session directories in {session_data_dir}')

    default_resample = 500
    remove_dynamic_only = False
    
    all_abs_points = []  # 保存所有session的绝对坐标点
    all_abs_intensities = []  # 保存所有session的强度值
    global_first_pose = None  # 全局基准first_pose
    for idx, session_dir in enumerate(session_dirs, 1):
        if(idx > 1):
            continue  # 只处理第一个session以测试

        print(f'\n\n{"#"*80}')
        print(f'# Session {idx}/{total_sessions}: session_dir = {session_dir}')
        print(f'{"#"*80}')
        pointclouds_dir = os.path.join(session_dir, 'pointclouds')
        labels_dir = os.path.join(session_dir, 'labels')
        poses_dir = os.path.join(session_dir, 'sparse/vehicle_geo_pose')

        # load_and_fit_pose_line(poses_dir)
        # exit(0)

        merge_result = merge_session_pointclouds(pointclouds_dir, labels_dir, poses_dir, resample=default_resample, remove_dynamic_only=remove_dynamic_only)

        print(f'\n\n{"#"*80}')
        first_pose = merge_result['first_pose'] if merge_result and 'first_pose' in merge_result else None
        print(f'\n\nfirst_pose: {first_pose}')
        # 记录第一个session的first_pose作为全局基准
        if global_first_pose is None and first_pose is not None:
            global_first_pose = first_pose.copy()
            print(f'全局基准first_pose: {global_first_pose}')
        # 打印绝对坐标下的前5个点
        if merge_result and 'points' in merge_result and first_pose is not None:
            abs_points = merge_result['points'] + first_pose
            all_abs_points.append(abs_points)
            # 保留原始强度信息
            if 'intensities' in merge_result:
                all_abs_intensities.append(merge_result['intensities'])
            # print("\n绝对坐标下前5个点:")
            # print(abs_points[:5])
        print(f'\n\n{"#"*80}')
    
    # 合并所有session的绝对坐标点和强度
    if all_abs_points and global_first_pose is not None:
        merged_abs_points = np.vstack(all_abs_points)
        # 所有点都减去第一个first_pose作为基准
        merged_rel_points = merged_abs_points - global_first_pose
        if all_abs_intensities:
            merged_intensities = np.hstack(all_abs_intensities)
        else:
            merged_intensities = np.zeros(len(merged_rel_points), dtype=np.float32)
        print(f"\n所有session合并后点数: {len(merged_rel_points)}")
        merged_pcd_path = "/data/all_sessions_merged_rel_points.pcd"
        save_merged_pointcloud_pcd(merged_rel_points, merged_intensities, merged_pcd_path)
        print(f"已保存所有session合并后的相对坐标点云: {merged_pcd_path}")
    exit(0)


if __name__ == "__main__":
    main()