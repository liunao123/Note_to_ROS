#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import numpy as np
import yaml
from pathlib import Path
from pypcd4 import PointCloud as PcdReader
import pcl
import laspy
import pyproj
import os
from pathlib import Path


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


def load_config(config_file: str = None) -> dict:
    """
    加载配置文件
    
    Args:
        config_file: 配置文件路径，默认为脚本所在目录的 config.yaml
        
    Returns:
        配置字典
    """
    if config_file is None:
        # 默认使用脚本所在目录的 config.yaml
        script_dir = Path(__file__).parent
        config_file = script_dir / "config.yaml"
    
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    print(f"已加载配置文件: {config_path}")
    return config

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
        
        # 设置强度 (LAS格式强度范围是0-65535)
        if intensities is not None:
            # 归一化强度到0-65535范围
            intensity_normalized = np.clip(intensities, 0, 255)
            las.intensity = (intensity_normalized * 257).astype(np.uint16)  # 0-255 -> 0-65535
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


def merge_session_pointclouds(pointclouds_dir, poses_dir, resample=1):
    """
    合并一个session中的所有点云到统一坐标系
    
    Args:
        pointclouds_dir: 点云文件目录
        poses_dir: 位姿文件目录
        resample: 采样率，1表示使用所有点云
        
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


def save_merged_pointcloud_pcd(points, intensities, output_file):
    """
    保存合并后的点云为PCD格式
    
    Args:
        points: 点云坐标 (N, 3)
        intensities: 强度值 (N,)
        output_file: 输出文件路径
        
    Returns:
        bool: 是否成功
    """
    print(f"\n保存PCD文件: {output_file}")
    
    # 创建输出目录
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    try:
        # 创建 PCL PointCloud_PointXYZI 对象
        cloud = pcl.PointCloud_PointXYZI()
        
        # 组合点坐标和强度信息
        points_with_intensity = np.column_stack([
            points[:, 0],      # x
            points[:, 1],      # y
            points[:, 2],      # z
            intensities        # intensity
        ]).astype(np.float32)
        
        # 设置点云数据
        cloud.from_array(points_with_intensity)
        
        # 保存为 PCD 文件
        pcl.save(cloud, output_file)
        
        print(f"  成功保存PCD文件!")
        print(f"  点数: {len(points)}")
        return True
        
    except Exception as e:
        print(f"  保存PCD文件失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def save_merged_pointcloud_las(points, intensities, first_pose, output_file, grid_size, epsg_code=32651):
    """
    保存合并后的点云为LAS格式
    
    Args:
        points: 相对坐标点云 (N, 3)
        intensities: 强度值 (N,)
        first_pose: UTM偏移量 [x, y, z]
        output_file: 输出文件路径
        epsg_code: EPSG代码
        
    Returns:
        bool: 是否成功
    """
    try:
        # 将相对坐标转换回绝对UTM坐标
        # absolute_points = points + first_pose
        # 网格大小
        # grid_size = 200.0  # 单位: 米
        # 计算每个点的网格索引
        min_x = points[:, 0].min()
        min_y = points[:, 1].min()
        grid_x = np.floor((points[:, 0] - min_x) / grid_size).astype(int)
        grid_y = np.floor((points[:, 1] - min_y) / grid_size).astype(int)
        grid_indices = list(zip(grid_x, grid_y))
        # 找到所有网格
        unique_grids = set(grid_indices)
        print(f"  网格划分: {len(unique_grids)} 个 {grid_size}x{grid_size}m 区块")
        base, ext = os.path.splitext(output_file)
        # 新建 grid_las 文件夹
        grid_dir = os.path.join(os.path.dirname(base), "grid_las")
        os.makedirs(grid_dir, exist_ok=True)
        total_points = 0
        for gx, gy in unique_grids:
            # 获取当前网格的掩码
            mask = (grid_x == gx) & (grid_y == gy)
            grid_points = points[mask]
            grid_intensities = intensities[mask]
            if len(grid_points) == 0:
                continue
            # 构建输出文件名到 grid_las 目录
            grid_filename = os.path.join(grid_dir, f"{os.path.basename(base)}_grid_{gx}_{gy}{ext}")
            absolute_points = grid_points + first_pose
            print(f"  保存网格: {grid_filename} 点数: {len(grid_points)}")
            success = write_las_file(grid_filename, absolute_points, grid_intensities, epsg_code=epsg_code)
            if success:
                total_points += len(grid_points)
            else:
                print(f"  网格 {gx},{gy} 保存失败!")
        print(f"  总计保存点数: {total_points}")
        return True
    except Exception as e:
        print(f"  保存LAS文件失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def save_merged_pointcloud_ply(points, intensities, output_file):
    """
    保存合并后的点云为PLY格式
    
    Args:
        points: 点云坐标 (N, 3)
        intensities: 强度值 (N,)
        output_file: 输出文件路径
        
    Returns:
        bool: 是否成功
    """
    
    # 创建输出目录
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    try:
        # 使用PCL保存PLY格式
        cloud = pcl.PointCloud_PointXYZI()
        
        # 组合点坐标和强度信息
        points_with_intensity = np.column_stack([
            points[:, 0],      # x
            points[:, 1],      # y
            points[:, 2],      # z
            intensities        # intensity
        ]).astype(np.float32)
        
        # 设置点云数据
        cloud.from_array(points_with_intensity)
        
        # 保存为 PLY 文件
        pcl.save(cloud, output_file, format='ply')
        print(f"\n保存PLY文件: {output_file}")
        print(f"  成功保存PLY文件!")
        print(f"  点数: {len(points)}")
        return True
        
    except Exception as e:
        print(f"  保存PLY文件失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def save_local_utm_offset(first_pose, output_file):
    """
    保存局部UTM偏移量到YAML文件
    
    Args:
        first_pose: UTM偏移量 [x, y, z]
        output_file: 输出YAML文件路径
        
    Returns:
        bool: 是否成功
    """
    print(f"\n保存局部UTM偏移量: {output_file}")
    
    # 创建输出目录
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    try:
        # 构建YAML数据
        offset_data = {
            'local_utm_offset': {
                'x': float(first_pose[0]),
                'y': float(first_pose[1]),
                'z': float(first_pose[2])
            },
            'description': '局部坐标系的UTM偏移量，用于将相对坐标转换为绝对UTM坐标',
            'note': 'absolute_utm = local_coordinates + offset'
        }
        
        # 保存到YAML文件
        with open(output_file, 'w', encoding='utf-8') as f:
            yaml.dump(offset_data, f, default_flow_style=False, allow_unicode=True)
        
        print(f"  成功保存UTM偏移量!")
        print(f"  X: {first_pose[0]:.3f}")
        print(f"  Y: {first_pose[1]:.3f}")
        print(f"  Z: {first_pose[2]:.3f}")
        return True
        
    except Exception as e:
        print(f"  保存UTM偏移量失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    # 加载配置
    config = load_config()
    merge_config = config.get('merge_session', {})
    
    # 从配置获取默认值
    input_config = merge_config.get('input', {})
    enable_specify_position = merge_config.get('enable_specify_position', True)
    output_config = merge_config.get('output', {})
    processing_config = merge_config.get('processing', {})

    default_project_dir = input_config.get('project_dir', './export')
    print(default_project_dir)
    
    default_output_dir = output_config.get('output_dir', './output')
    default_output_filename = output_config.get('output_filename', 'merged_map')  # 移除.pcd扩展名
    default_resample = processing_config.get('resample', 10)
    
    # 从配置文件获取经纬度，用于自动计算EPSG
    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    default_longitude = default_location.get('longitude', 120.65582103)
    default_latitude = default_location.get('latitude', 31.41033324)
    

    # 只从 config.yaml 读取参数
    project_dir = input_config.get('project_dir', './export')
    output_base = os.path.join(output_config.get('output_dir', './output'), output_config.get('output_filename', 'merged_map'))
 
    save_las = output_config.get('save_las', True)
    las_grid_size = output_config.get('las_grid_size', 200.0)

    save_ply = output_config.get('save_ply', True)
    save_pcd = output_config.get('save_pcd', True)
    enable_filter = processing_config.get('enable_filter', True)


    # print(f"enable_filter: {enable_filter}")
    print(f"save_las: {save_las}")
    print(f"save_pcd: {save_pcd}")
    print(f"save_ply: {save_ply}")
    print(f"default_resample: {default_resample}")

    # 自动计算EPSG代码
    epsg = calculate_utm_epsg(default_longitude, default_latitude)
    print(f"根据经纬度自动计算EPSG代码: {epsg} (经度: {default_longitude}, 纬度: {default_latitude})")
    
    # 检查输入目录
    if not project_dir:
        print("错误: 未指定输入项目目录")
        print("请在配置文件中设置 merge_session.input.project_dir 或使用 -i 参数指定")
        sys.exit(1)

    filter_distance = search_params.get('distance', 100.0)
    if enable_specify_position:
        enable_filter = True
        project_dir = default_project_dir + f"/search_{default_latitude:.6f}_{default_longitude:.6f}_{filter_distance}m"
    else:
        enable_filter = False
        print("不指定搜索位置，禁用空间过滤，使用完整点云合并，上述指定的位置参数将被忽略")

    pointclouds_subdir = input_config.get('pointclouds_subdir', 'pointclouds')
    poses_subdir = input_config.get('poses_subdir', 'sparse/vehicle_geo_pose')
    
    pointclouds_dir = os.path.join(project_dir, pointclouds_subdir)
    poses_dir = os.path.join(project_dir, poses_subdir)
    
    # 检查输入目录是否存在
    if not os.path.exists(pointclouds_dir):
        print(f"错误: 点云目录不存在: {pointclouds_dir}")
        sys.exit(1)
    
    if not os.path.exists(poses_dir):
        print(f"错误: 位姿目录不存在: {poses_dir}")
        sys.exit(1)
    
    # 打印配置信息
    print("\n" + "="*80)
    print("  点云合并工具")
    print("="*80)
    print(f"输入配置:")
    print(f"  项目目录: {project_dir}")
    print(f"  点云目录: {pointclouds_dir}")
    print(f"  位姿目录: {poses_dir}")
    print(f"\n输出配置:")
    print(f"  输出基础路径: {output_base}")
    if save_pcd:
        print(f"  PCD文件: {output_base}.pcd")
    if save_las:
        print(f"  LAS文件: {output_base}.las")
        print(f"  EPSG代码: {epsg}")
    if save_ply:
        print(f"  PLY文件: {output_base}.ply")
    print(f"\n处理参数:")
    print(f"  采样率: {default_resample} (每{default_resample}个点云使用1个)")
    if enable_filter:
        filter_dist = search_params.get('distance', 100.0)
        print(f"  空间过滤: 启用 (中心: {default_latitude:.6f}, {default_longitude:.6f}, 范围: ±{filter_dist}m)")
    else:
        print(f"  空间过滤: 禁用")
    print("="*80 + "\n")
    
    # 执行合并
    merge_result = merge_session_pointclouds(pointclouds_dir, poses_dir, resample=default_resample)
    
    if merge_result is None:
        print("点云合并失败!")
        sys.exit(1)
    
    # 提取合并结果
    points = merge_result['points']
    intensities = merge_result['intensities']
    first_pose = merge_result['first_pose']
    processed_count = merge_result['processed_count']
    error_count = merge_result['error_count']
    
    print(f"\n合并完成:")
    print(f"  成功处理: {processed_count} 个文件")
    print(f"  失败: {error_count} 个文件")
    print(f"  总点数: {len(points)}")
    
    # 根据搜索位置过滤点云
    if enable_filter:
        if filter_distance and filter_distance > 0:
            utm_zone = calculate_utm_zone(default_longitude)
            points, intensities = filter_pointcloud_by_location(
                points, 
                intensities, 
                first_pose,
                default_longitude,
                default_latitude,
                filter_distance,
                utm_zone
            )
            
            if len(points) == 0:
                print("\n错误: 过滤后没有剩余点云!")
                sys.exit(1)
    else:
        print("\n跳过空间过滤（--no-filter）")    # 保存点云文件
    print("\n" + "="*80)
    print("保存文件")
    print("="*80)
    
    success_count = 0
    total_count = 0
    
    # 保存LAS格式（使用绝对坐标）
    if save_las:
        total_count += 1
        las_path = output_base + '.las'
        if save_merged_pointcloud_las(points, intensities, first_pose, las_path, las_grid_size, epsg_code=epsg):
            success_count += 1

    # 保存PCD格式（使用相对坐标）
    if save_pcd:
        total_count += 1
        pcd_path = output_base + '_local.pcd'
        if save_merged_pointcloud_pcd(points, intensities, pcd_path):
            success_count += 1
    
    # 保存PLY格式（使用相对坐标）
    if save_ply:
        total_count += 1
        ply_path = output_base + '_local.ply'
        if save_merged_pointcloud_ply(points, intensities, ply_path):
            success_count += 1

    # 如果保存了相对坐标的文件（PCD或PLY），保存UTM偏移量
    if save_ply or save_pcd:
        print("\n" + "="*80)
        yaml_path = output_base + '_local_utm.yaml'
        if save_local_utm_offset(first_pose, yaml_path):
            print(f"\n提示: 相对坐标文件(_local.pcd/_local.ply)需要配合 {os.path.basename(yaml_path)} 使用")
            print(f"      绝对坐标 = 相对坐标 + UTM偏移量")
        print("="*80)


    
    # 输出最终结果
    print("\n" + "="*80)
    if success_count == total_count and total_count > 0:
        print(f"✓ 点云合并和保存成功完成! ({success_count}/{total_count} 个格式)")
    elif success_count > 0:
        print(f"⚠ 点云合并完成，但部分格式保存失败 ({success_count}/{total_count} 个格式)")
    else:
        print("✗ 所有格式保存失败!")
    print("="*80)


if __name__ == "__main__":
    main()
