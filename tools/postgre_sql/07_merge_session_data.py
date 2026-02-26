#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import numpy as np
import yaml
import json
from pathlib import Path
from pypcd4 import PointCloud as PcdReader
import pcl
import laspy
import pyproj

# 使用qt.load_config的统一配置加载
from qt.load_config import load_config

# --- 新增：点云写KML函数 ---
def save_pointcloud_to_kml(points, first_pose, output_file, utm_zone):
    """
    将点云保存为KML文件，颜色根据Z值渲染（伪彩色）
    Args:
        points: 相对坐标点云 (N, 3)
        first_pose: UTM偏移量 [x, y, z]
        output_file: 输出KML文件路径
        utm_zone: UTM区号（int）
    Returns:
        bool: 是否成功
    """
    import pyproj
    try:
        abs_points = points + first_pose
        utm_srid = 32600 + utm_zone
        utm = pyproj.CRS(f'EPSG:{utm_srid}')
        wgs84 = pyproj.CRS('EPSG:4326')
        transformer = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
        lons, lats = transformer.transform(abs_points[:, 0], abs_points[:, 1])
        zs = abs_points[:, 2]
        z_min, z_max = float(zs.min()), float(zs.max())
        if z_max > z_min:
            z_norm = (zs - z_min) / (z_max - z_min)
        else:
            z_norm = np.zeros_like(zs)
        def colormap(val):
            # rainbow色带（HSV->RGB），val: 0~1
            import colorsys
            val = np.clip(val, 0, 1)
            h = (1.0 - val) * 0.7  # 0=red, 0.7=blue (反向)
            r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
            return int(r*255), int(g*255), int(b*255)
        # 限制最多10000个点，均匀采样
        max_points = 1000
        total_points = len(lons)
        if total_points > max_points:
            idxs = np.linspace(0, total_points - 1, max_points, dtype=int)
        else:
            idxs = np.arange(total_points)

        kml_colors = [f"ff{b:02x}{g:02x}{r:02x}" for r, g, b in map(colormap, z_norm[idxs])]

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
            f.write('  <Document>\n')
            for i, (idx, color) in enumerate(zip(idxs, kml_colors)):
                f.write(f'    <Style id="point_style_{i}">\n')
                f.write(f'      <IconStyle><color>{color}</color><scale>0.5</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon></IconStyle>\n')
                f.write('    </Style>\n')
            for i, (idx, color) in enumerate(zip(idxs, kml_colors)):
                f.write('    <Placemark>\n')
                f.write(f'      <styleUrl>#point_style_{i}</styleUrl>\n')
                f.write('      <Point>\n')
                f.write(f'        <coordinates>{lons[idx]},{lats[idx]},{zs[idx]:.3f}</coordinates>\n')
                f.write('      </Point>\n')
                f.write('    </Placemark>\n')
            f.write('  </Document>\n')
            f.write('</kml>\n')
        print(f"\n保存KML文件: {output_file} (点数: {len(idxs)})")
        return True
    except Exception as e:
        print(f"保存KML文件失败: {e}")
        import traceback
        traceback.print_exc()
        return False


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
            out_boxs = load_3dbox_from_json(label_file)
            if len(out_boxs) > 0:
                # print(f"  Loaded {len(out_boxs)} boxes from {label_file}")
                filtered_points, filtered_intensities = remove_points_within_3dbox(
                    filtered_points, filtered_intensities, out_boxs, remove_dynamic_only=remove_dynamic_only
                )
            
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
        # 新建 grid_las 文件夹
        grid_dir = os.path.join( output_file + "/grid_las/")
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
            grid_filename = os.path.join(grid_dir, f"grid_{gx}_{gy}.las")
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
            'note': 'absolute_utm = local_coordinates + local_utm_offset'
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
    enable_specify_position = merge_config.get('enable_specify_position', True)
    
    processing_config = merge_config.get('processing', {})
    default_resample = processing_config.get('resample', 10)
    remove_dynamic_only = processing_config.get('remove_dynamic_only', False)
    enable_filter = processing_config.get('enable_filter', True)

    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    # 搜索参数 复用 search_params 中的默认位置
    target_lat = default_location.get('latitude', 31.41033324)
    target_lon = default_location.get('longitude', 120.65582103)
    roi_distance_max = search_params.get('roi_distance_max', 100.0)
    roi_distance_min = search_params.get('roi_distance_min', 50.0)
    
    # 复用
    # 输出目录和会话名模板直接从 search_params 读取
    output_root = Path(search_params.get('export_dir', '/mnt/nvme0n2/project/postgresql/export/'))
    session_name_template = search_params.get('session_name_template', 'search_{lat:.6f}_{lon:.6f}_{roi_distance_min}_{roi_distance_max}m')
    session_name = session_name_template.format(lat=target_lat, lon=target_lon, roi_distance_min=roi_distance_min, roi_distance_max=roi_distance_max)
    default_project_dir = output_root / session_name

    if enable_specify_position:
        enable_filter = True
        project_dir = default_project_dir / f"/search_{target_lat:.6f}_{target_lon:.6f}_{roi_distance_max}m"
    else:
        enable_filter = False
        project_dir = default_project_dir
        session_dir = merge_config.get('session_dir', {})
        default_project_dir = Path(session_dir)
        print(f" enable_specify_position is false, use dir: merge_session/session_dir ")
        print("不指定搜索位置，禁用空间过滤，使用完整点云合并，上述指定的位置参数将被忽略")

    # 检查 default_project_dir 是否存在
    if not default_project_dir.exists():
        print(f"错误: default_project_dir 不存在: {default_project_dir}")
        sys.exit(1)
    print(default_project_dir)
    # return
    
    # default_project_dir="/data/dwm_data/park_20251120_0/"
    work_dir = Path(default_project_dir) 
    print(f"输入目录 : {work_dir}")

    # 只从 config.yaml 读取参数
    output_config = merge_config.get('output', {})
    output_base = os.path.join(output_config.get('output_dir', './output'), 
    output_config.get('output_filename', 'merged_map'))
 
    save_las = output_config.get('save_las', True)
    las_grid_size = output_config.get('las_grid_size', 200.0)

    save_ply = output_config.get('save_ply', True)
    save_pcd = output_config.get('save_pcd', True)

    # print(f"enable_filter: {enable_filter}")
    print(f"save_las: {save_las}")
    print(f"save_pcd: {save_pcd}")
    print(f"save_ply: {save_ply}")
    print(f"default_resample: {default_resample}")
    print(f"remove_dynamic_only: {remove_dynamic_only}")

    # 自动计算EPSG代码
    epsg = calculate_utm_epsg(target_lon, target_lat)
    print(f"根据经纬度自动计算EPSG代码: {epsg} (经度: {target_lon}, 纬度: {target_lat})")

    pointclouds_dir = os.path.join(default_project_dir, 'pointclouds')
    labels_dir = os.path.join(default_project_dir, 'labels')
    poses_dir = os.path.join(default_project_dir, 'sparse/vehicle_geo_pose')
    
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
        print(f"  空间过滤: 启用 (中心: {target_lat:.6f}, {target_lon:.6f}, 范围: ±{roi_distance_max}m)")
    else:
        print(f"  空间过滤: 禁用")
    print("="*80 + "\n")
    
    # 执行合并
    merge_result = merge_session_pointclouds(pointclouds_dir, labels_dir, poses_dir, resample=default_resample, remove_dynamic_only=remove_dynamic_only)
    
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
        if roi_distance_max and roi_distance_max > 0:
            utm_zone = calculate_utm_zone(target_lon)
            points, intensities = filter_pointcloud_by_location(
                points, 
                intensities, 
                first_pose,
                target_lon,
                target_lat,
                roi_distance_max,
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
        las_path = output_base  
        if save_merged_pointcloud_las(points, intensities, first_pose, las_path, las_grid_size, epsg_code=epsg):
            success_count += 1

    # 保存PCD格式（使用相对坐标）
    if save_pcd:
        total_count += 1
        pcd_path = output_base + '/local_map.pcd'
        if save_merged_pointcloud_pcd(points, intensities, pcd_path):
            success_count += 1
    
    # 保存PLY格式（使用相对坐标）
    if save_ply:
        total_count += 1
        ply_path = output_base + '/local_map.ply'
        if save_merged_pointcloud_ply(points, intensities, ply_path):
            success_count += 1

    # 如果保存了相对坐标的文件（PCD或PLY），保存UTM偏移量
    if save_ply or save_pcd:
        print("\n" + "="*80)
        yaml_path = output_base + '/local_utm.yaml'
        if save_local_utm_offset(first_pose, yaml_path):
            print(f"\n提示: 相对坐标文件(_local.pcd/_local.ply)需要配合 {os.path.basename(yaml_path)} 使用")
            print(f"      绝对坐标 = 相对坐标 + UTM偏移量")
        print("="*80)

    # 保存KML格式（绝对坐标，Z值渲染）
    # utm_zone = calculate_utm_zone(target_lon)
    # kml_path = output_base + '/pointcloud_z.kml'
    # save_pointcloud_to_kml(points, first_pose, kml_path, utm_zone)
    
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