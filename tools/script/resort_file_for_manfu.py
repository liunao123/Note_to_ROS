#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件整理脚本 - 遍历和处理PCD文件
"""

import os
import glob
import json
import yaml
import numpy as np
import shutil
from pathlib import Path

# 全局变量定义

WORK_DIRECTORY = "/mnt/nvme0n1p2/project/tag/251120/"
EXTRINSICS_DIR = "/mnt/nvme0n1p2/project/tag/251120/calib/extrinsics"
INTRINSICS_DIR = "/mnt/nvme0n1p2/project/tag/251120/calib/new_intrinsics"

OUTPUT_DIRECTORY = "/mnt/nvme0n1p2/project/tag/test"

def read_camera_intrinsics(camera_name):
    """
    读取相机内参文件
    
    Args:
        camera_name (str): 相机名称 (如: cam1, cam2, etc.)
    
    Returns:
        dict: 包含相机内参的字典，如果读取失败返回None
    """
    global INTRINSICS_DIR
    intrinsic_file = os.path.join(INTRINSICS_DIR, f"{camera_name}_Intrinsic.yaml")
    
    if not os.path.exists(intrinsic_file):
        print(f"警告: 找不到内参文件 {intrinsic_file}")
        return None
    
    try:
        # 读取文件内容并手动解析OpenCV YAML格式
        with open(intrinsic_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 提取基本信息
        image_width = 1920
        image_height = 1200
        camera_matrix = []
        dist_coeffs = []
        
        # 使用正则表达式提取数据
        import re
        
        # 提取ImageWidth
        width_match = re.search(r'ImageWidth:\s*(\d+)', content)
        if width_match:
            image_width = int(width_match.group(1))
        
        # 提取ImageHeight
        height_match = re.search(r'ImageHeight:\s*(\d+)', content)
        if height_match:
            image_height = int(height_match.group(1))
        
        # 提取相机矩阵数据
        camera_matrix_match = re.search(r'cameraMatrix:.*?data:\s*\[(.*?)\]', content, re.DOTALL)
        if camera_matrix_match:
            matrix_data = camera_matrix_match.group(1)
            # 清理数据并转换为浮点数列表
            matrix_values = re.findall(r'[+-]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][+-]?\d+)?', matrix_data)
            camera_matrix = [float(x) for x in matrix_values]
        
        # 提取畸变系数数据
        dist_coeffs_match = re.search(r'distCoeffs:.*?data:\s*\[(.*?)\]', content, re.DOTALL)
        if dist_coeffs_match:
            dist_data = dist_coeffs_match.group(1)
            # 清理数据并转换为浮点数列表
            dist_values = re.findall(r'[+-]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][+-]?\d+)?', dist_data)
            dist_coeffs = [float(x) for x in dist_values]
        
        # 将相机矩阵重新组织为3x4矩阵格式
        if len(camera_matrix) == 9:
            internal_matrix = [
                [camera_matrix[0], camera_matrix[1], camera_matrix[2], 0],
                [camera_matrix[3], camera_matrix[4], camera_matrix[5], 0],
                [camera_matrix[6], camera_matrix[7], camera_matrix[8], 0]
            ]
        else:
            # 如果无法解析，使用默认值
            print(f"警告: 无法解析相机矩阵，使用默认值。相机: {camera_name}")
            internal_matrix = [
                [1657.0318603515625, 0, 966.8290405273438, 0],
                [0, 1823.3201904296875, 540.2271118164062, 0],
                [0, 0, 1, 0]
            ]
        
        # print(f"成功读取 {camera_name} 内参: 宽度={image_width}, 高度={image_height}")
        
        return {
            'width': image_width,
            'height': image_height,
            'internal': internal_matrix,
            'distCoeffs': dist_coeffs
        }
    
    except Exception as e:
        print(f"读取内参文件失败: {intrinsic_file}, 错误: {e}")
        return None


def quaternion_to_rotation_matrix(w, x, y, z):
    """
    将四元数转换为旋转矩阵
    
    Args:
        w, x, y, z: 四元数的四个分量 (w为实部)
    
    Returns:
        numpy.ndarray: 3x3旋转矩阵
    """
    # 归一化四元数
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # 计算旋转矩阵
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    
    return R


def opencv_matrix_constructor(loader, node):
    """自定义YAML构造器，用于处理OpenCV矩阵标签"""
    mapping = loader.construct_mapping(node)
    return mapping


def read_extrinsics(sensor_name):
    """
    读取指定传感器的外参文件
    
    Args:
        sensor_name (str): 传感器名称，如 'cam1', 'cam2', 'hesai128' 等
    
    Returns:
        tuple: (rotation_matrix, translation_vector) 或 (None, None) 如果读取失败
    """
    
    # 构建文件名映射
    file_mapping = {
        'hesai128': '0_hesai128_2_vehicle_Extrinsics.yaml',
        'cam1': '1_cam1_2_vehicle_Extrinsics.yaml',
        'cam2': '2_cam2_2_vehicle_Extrinsics.yaml',
        'cam3': '3_cam3_2_vehicle_Extrinsics.yaml',
        'cam4': '4_cam4_2_vehicle_Extrinsics.yaml',
        'cam5': '5_cam5_2_vehicle_Extrinsics.yaml',
        'cam6': '6_cam6_2_vehicle_Extrinsics.yaml',
        'cam7': '7_cam7_2_vehicle_Extrinsics.yaml',
        'cgi830': '8_cgi830_2_vehicle_Extrinsics.yaml'
    }
    
    if sensor_name not in file_mapping:
        print(f"警告: 未找到传感器 {sensor_name} 的外参文件映射")
        return None
    global EXTRINSICS_DIR
    extrinsics_file = os.path.join(EXTRINSICS_DIR, file_mapping[sensor_name])
    
    if not os.path.exists(extrinsics_file):
        print(f"警告: 外参文件不存在: {extrinsics_file}")
        return None
    
    try:
        # 创建自定义的YAML加载器
        class CustomLoader(yaml.SafeLoader):
            pass
        
        # 添加自定义构造器来处理OpenCV矩阵标签
        CustomLoader.add_constructor('tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)
        
        with open(extrinsics_file, 'r', encoding='utf-8') as f:
            content = f.read()
            # 移除YAML版本标识符，因为它会导致解析问题
            if content.startswith('%YAML:'):
                lines = content.split('\n')
                # 跳过前两行（%YAML:1.0 和 ---）
                content = '\n'.join(lines[2:])
            
            # 使用自定义加载器
            yaml_content = yaml.load(content, Loader=CustomLoader)
        
        # 读取四元数 (w, x, y, z)
        quaternion = yaml_content['r_quaternion_wxyz']['data']
        w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        
        # 读取平移向量
        translation = yaml_content['t_metric_xyz']['data']
        t_vector = [translation[0], translation[1], translation[2]]
        
        # 将四元数转换为旋转矩阵
        rotation_matrix = quaternion_to_rotation_matrix(w, x, y, z)
        
        # print(f"成功读取 {sensor_name} 外参")
        # print(f"  四元数 (w, x, y, z): {w:.6f}, {x:.6f}, {y:.6f}, {z:.6f}")
        # print(f"  旋转矩阵:")
        # for row in rotation_matrix:
        #     print(f"    [{row[0]:10.6f}, {row[1]:10.6f}, {row[2]:10.6f}]")
        # print(f"  平移向量: [{t_vector[0]:10.6f}, {t_vector[1]:10.6f}, {t_vector[2]:10.6f}]")
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = t_vector
        return transform
    
    except Exception as e:
        print(f"读取外参文件失败 {extrinsics_file}: {e}")
        return None


def create_camera_json_file(json_filepath, camera_name):
    """
    创建相机标定JSON文件
    
    Args:
        json_filepath (str): JSON文件的完整路径
        camera_name (str): 相机名称
    
    Returns:
        bool: 是否成功创建文件
    """
    # 如果文件已存在，则不重复创建
    if os.path.exists(json_filepath):
        return False
    
    # 读取对应相机的内参
    intrinsics = read_camera_intrinsics(camera_name)
    
    # 读取对应相机的外参
    T_v_c = read_extrinsics(camera_name)
    T_v_l = read_extrinsics("hesai128")

    # 检查外参是否读取成功
    if T_v_c is not None and T_v_l is not None:
        # 计算从相机到雷达的变换矩阵: T_c_l = T_v_c^(-1) * T_v_l
        T_c_l = np.linalg.inv(T_v_c) @ T_v_l
        rotation_matrix = T_c_l[:3, :3]
        translation_vector = T_c_l[:3, 3]
    else:
        rotation_matrix = None
        translation_vector = None    

    # 打印旋转矩阵和平移向量
    # print(f"=== {camera_name} 外参数据 ===")
    # if rotation_matrix is not None and translation_vector is not None:
    #     print(f"旋转矩阵 (rotation_matrix):")
    #     print(rotation_matrix)
    #     print(f"平移向量 (translation_vector):")
    #     print(translation_vector)
    # else:
    #     print("外参读取失败，将使用默认值")
    # print("=" * 30)
    
    # 如果外参读取成功，使用读取的数据；否则使用默认值
    if rotation_matrix is not None and translation_vector is not None:
        r_data = rotation_matrix.tolist()
        t_data = translation_vector.tolist()  # 确保转换为Python列表
    else:
        # 默认的外参数据（旋转和平移）
        r_data = [
            [
                -0.8986015319824219,
                -0.4384683668613434,
                0.01615249365568161
            ],
            [
                -0.08524826169013977,
                0.1383599042892456,
                -0.9867062568664551
            ],
            [
                0.4304046928882599,
                -0.8880327343940735,
                -0.16170905530452728
            ]
        ]
        
        t_data = [
            -2.135498046875,
            0.01700592041015625,
            35.81494140625
        ]
    
    # 构建JSON内容
    if intrinsics:
        json_content = {
            "width": intrinsics['width'],
            "height": intrinsics['height'],
            "r": r_data,
            "t": t_data,
            "internal": intrinsics['internal']
        }
        # 如果有畸变系数，也添加进去
        if intrinsics.get('distCoeffs'):
            json_content["distCoeffs"] = intrinsics['distCoeffs']
    else:
        # 如果无法读取内参，使用默认值
        json_content = {
            "width": 1920,
            "height": 1200,
            "r": r_data,
            "t": t_data,
            "internal": [
                [1657.0318603515625, 0, 966.8290405273438, 0],
                [0, 1823.3201904296875, 540.2271118164062, 0],
                [0, 0, 1, 0]
            ]
        }
    
    try:
        with open(json_filepath, 'w', encoding='utf-8') as f:
            json.dump(json_content, f, indent=2, ensure_ascii=False)
        return True
    except Exception as e:
        print(f"创建JSON文件失败: {json_filepath}, 错误: {e}")
        return False


def find_pcd_files(directory):
    """
    遍历指定目录及其子目录，查找所有PCD文件
    
    Args:
        directory (str): 要搜索的目录路径
    
    Returns:
        list: PCD文件路径列表，按文件名排序
    """
    pcd_files = []
    
    # 使用glob模式匹配查找所有PCD文件
    pattern = os.path.join(directory, "**", "*.pcd")
    pcd_files = glob.glob(pattern, recursive=True)
    
    # 按文件名进行排序，确保顺序一致
    def extract_number(filepath):
        """从文件路径中提取数字用于排序"""
        filename = os.path.basename(filepath)
        try:
            # 假设文件名格式为: 数字_时间戳.pcd
            return int(filename.split('_')[0])
        except (ValueError, IndexError):
            # 如果无法提取数字，使用文件名排序
            return float('inf')
    
    pcd_files.sort(key=extract_number)
    
    return pcd_files


def print_pcd_files(directory, output_directory, images_directory, subfolder_names=None):
    """
    打印指定目录下所有PCD文件的信息，并创建对应的输出文件夹，按分组组织
    
    Args:
        directory (str): 要搜索的目录路径
        output_directory (str): 输出目录路径
        images_directory (str): 图像文件目录路径
        subfolder_names (list): 要在每个PCD文件夹中创建的子文件夹名称列表
    """
    print(f"正在搜索目录: {directory}")
    print(f"输出目录: {output_directory}")
    print("-" * 50)
    
    pcd_files = find_pcd_files(directory)
    
    if not pcd_files:
        print("未找到任何PCD文件")
        return
    
    print(f"找到 {len(pcd_files)} 个PCD文件:")
    print()
    
    # 确保输出目录存在
    os.makedirs(output_directory, exist_ok=True)
    
    # 每组的大小
    group_size = 100

    cnt = 0
    for i, pcd_file in enumerate(pcd_files, 1):
        cnt += 1
        print(pcd_file)
        if cnt > 150:
            break

        # 获取文件名（不含路径）
        filename = os.path.basename(pcd_file)
        # 获取相对于搜索目录的路径
        relative_path = os.path.relpath(pcd_file, directory)
        
        # 提取relative_path的不含后缀的部分
        relative_path_no_ext = os.path.splitext(relative_path)[0]
        
        # 从文件名中提取数字部分（假设格式为 数字_时间戳.pcd）
        file_number = int(filename.split('_')[0])
        
        # 计算当前文件应该属于哪个组（0-99为group_1, 100-199为group_2, 以此类推）
        group_number = file_number // group_size + 1
        group_folder_name = f"group_{group_number}"
        
        # 创建分组文件夹路径
        group_folder_path = os.path.join(output_directory, group_folder_name)
        os.makedirs(group_folder_path, exist_ok=True)
        
        # 结合group文件夹创建对应的文件夹路径
        target_folder_path = os.path.join(group_folder_path, relative_path_no_ext)
        
        # 创建目标文件夹
        os.makedirs(target_folder_path, exist_ok=True)
        
        # 如果指定了子文件夹名称，则在目标文件夹中创建这些子文件夹
        created_subfolders = []
        created_json_files = []
        copied_pcd_file = False
        copied_image_files = []
        if subfolder_names:
            for subfolder_name in subfolder_names:
                subfolder_path = os.path.join(target_folder_path, str(subfolder_name))
                os.makedirs(subfolder_path, exist_ok=True)
                created_subfolders.append(subfolder_name)
                
                # 如果是lidar文件夹，复制对应的PCD文件
                if str(subfolder_name) == 'lidar':
                    try:
                        # 构建目标PCD文件路径
                        target_pcd_path = os.path.join(subfolder_path, filename)
                        
                        # 如果目标文件不存在，则复制
                        if not os.path.exists(target_pcd_path):
                            shutil.copy2(pcd_file, target_pcd_path)
                            copied_pcd_file = True
                            # print(f"     已复制PCD文件: {filename} -> {subfolder_path}")
                        else:
                            print(f"     PCD文件已存在，跳过复制: {target_pcd_path}")
                    except Exception as e:
                        print(f"     复制PCD文件失败: {e}")
                
                # 如果是cam文件夹，在其中创建同名的JSON文件并复制对应的图像文件
                elif str(subfolder_name).startswith('cam'):
                    json_filename = f"{subfolder_name}.json"
                    json_filepath = os.path.join(subfolder_path, json_filename)
                    
                    # 调用专门的函数创建JSON文件
                    if create_camera_json_file(json_filepath, str(subfolder_name)):
                        created_json_files.append(json_filename)
                    
                    # 复制对应的图像文件
                    if images_directory and os.path.exists(images_directory):
                        cam_images_folder = os.path.join(images_directory, str(subfolder_name))
                        if os.path.exists(cam_images_folder):
                            # 构建图像文件名（基于PCD文件名）
                            # PCD文件名格式：数字_时间戳.pcd
                            # 源图像文件名格式：数字_时间戳_cam编号.jpg
                            # 目标图像文件名格式：数字_时间戳.jpg（去除cam编号）
                            pcd_name_parts = os.path.splitext(filename)[0]  # 去掉.pcd后缀
                            source_image_filename = f"{pcd_name_parts}_{subfolder_name}.jpg"
                            target_image_filename = f"{pcd_name_parts}.jpg"  # 去除相机名字
                            source_image_path = os.path.join(cam_images_folder, source_image_filename)
                            target_image_path = os.path.join(subfolder_path, target_image_filename)
                            
                            try:
                                if os.path.exists(source_image_path) and not os.path.exists(target_image_path):
                                    shutil.copy2(source_image_path, target_image_path)
                                    copied_image_files.append(target_image_filename)
                                    # print(f"     已复制图像文件: {target_image_filename} -> {subfolder_path}")
                                elif not os.path.exists(source_image_path):
                                    print(f"     警告: 图像文件不存在: {source_image_path}")
                                else:
                                    print(f"     图像文件已存在，跳过复制: {target_image_path}")
                            except Exception as e:
                                print(f"     复制图像文件失败: {e}")
                        else:
                            print(f"     警告: 相机图像文件夹不存在: {cam_images_folder}")
        
        print(f"{i:3d}. 文件名: {filename}")
        # print(f"     文件编号: {file_number}")
        # print(f"     相对路径: {relative_path}")
        # print(f"     不含后缀的相对路径: {relative_path_no_ext}")
        # print(f"     完整路径: {pcd_file}")
        print(f"     分组: {group_folder_name}")
        # print(f"     创建的目标文件夹: {target_folder_path}")
        # if created_subfolders:
        #     print(f"     创建的子文件夹: {created_subfolders}")
        # if copied_pcd_file:
        #     print(f"     已复制PCD文件到lidar文件夹")
        # if created_json_files:
        #     print(f"     创建的JSON文件: {created_json_files}")
        # if copied_image_files:
        #     print(f"     已复制图像文件: {copied_image_files}")
        print()


def main():
    """
    主函数
    """
    # 使用全局变量定义的目录
    global OUTPUT_DIRECTORY, WORK_DIRECTORY
    output_directory = OUTPUT_DIRECTORY
    work_directory = WORK_DIRECTORY
    search_directory = work_directory + "/pointclouds"
    images_directory = work_directory + "/images"
    
    # 指定要在每个PCD文件夹中创建的子文件夹名称
    # 可以根据需要修改这个数组
    subfolder_names = ["lidar", "cam1", "cam2", "cam3", "cam4", "cam5", "cam6", "cam7"]  # 示例：创建名为 lidar, camera, imu 的子文件夹

    # 如果需要搜索特定目录，可以修改这里
    # search_directory = "/path/to/your/directory"
    
    # 也可以从命令行参数获取目录
    import sys
    if len(sys.argv) > 1:
        search_directory = sys.argv[1]
    
    # 检查目录是否存在
    if not os.path.exists(search_directory):
        print(f"错误: 目录 '{search_directory}' 不存在")
        return
    
    if not os.path.isdir(search_directory):
        print(f"错误: '{search_directory}' 不是一个目录")
        return
    
    print(f"将在每个PCD文件夹中创建子文件夹: {subfolder_names}")
    print()
    
    # 执行PCD文件搜索和打印
    print_pcd_files(search_directory, output_directory, images_directory, subfolder_names)


if __name__ == "__main__":
    main()