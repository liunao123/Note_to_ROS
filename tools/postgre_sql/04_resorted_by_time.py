#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import argparse
from pathlib import Path
import shutil
import re
import numpy as np
import yaml
import json
import laspy
import pyproj
# 使用qt.load_config的统一配置加载
from qt.load_config import load_config

# 导入3D box投影工具函数
from qt.box_projection import (
    load_lidar_pose_from_file,
    lonlat_to_utm,
    utm_to_lonlat
)

def parse_id_time(filename):
    # 支持 id_time_camX.jpg 或 id_time.jpg
    # 通过下划线个数判断类型
    name, dot, ext = filename.rpartition('.')
    if not dot:
        return None
    parts = name.split('_')
    if len(parts) == 3:
        # id_time_cam
        try:
            id_ = int(parts[0])
            t = float(parts[1])
            cam = parts[2]
            return id_, t, cam, '.' + ext
        except Exception:
            return None
    elif len(parts) == 2:
        # id_time
        try:
            id_ = int(parts[0])
            t = float(parts[1])
            return id_, t, '', '.' + ext
        except Exception:
            return None
    return None

def process_folder(src_dir, dst_dir):
    files = [f for f in os.listdir(src_dir) if os.path.isfile(os.path.join(src_dir, f))]
    parsed = []
    for f in files:
        res = parse_id_time(f)
        if res:
            id_, t, cam, ext = res
            parsed.append((id_, t, cam, ext, f, src_dir))
    # 按时间戳排序
    parsed.sort(key=lambda x: x[1])
    # 重新命名并复制
    for new_id, (old_id, t, cam, ext, fname, folder) in enumerate(parsed):
        t_str = f"{t:.3f}"
        # 恢复原始风格
        if cam:
            new_name = f"{new_id}_{t_str}_{cam}{ext}"
        else:
            new_name = f"{new_id}_{t_str}{ext}"
        shutil.copy2(os.path.join(folder, fname), os.path.join(dst_dir, new_name))
    print(f"  {src_dir}: {len(parsed)} files")
    return len(parsed)


def write_pose_to_yaml(yaml_file, transformation_matrix, offset_utm, timestamp):
    """
    将位姿数据写入YAML文件。
    transformation_matrix: 4x4 numpy数组
    offset_utm: 长度为3的numpy数组
    timestamp: 浮点数
    """
    # 使用OrderedDict强制写入顺序：timestamp, pose_utm, offset_utm
    # 直接用字符串格式化输出，保证顺序和精度
    def format_float(val):
        # 保证小数点后15位有效数字，不丢失精度
        s = f"{val:.12f}"
        # 去除多余的0但保留小数点后精度
        s = s.rstrip('0').rstrip('.') if '.' in s else s
        # 如果是科学计数法，保留原始格式
        if 'e' in s or 'E' in s:
            s = f"{val:.15g}"
        return s
    
    def format_timestamp(val):
        # timestamp强制保留一位小数
        return f"{val:.1f}"

    with open(yaml_file, 'w') as f:
        f.write(f"timestamp: {format_timestamp(timestamp)}\n")
        f.write("pose_utm:\n")
        for v in transformation_matrix.reshape(-1):
            f.write(f"  - {format_float(v)}\n")
        f.write("offset_utm:\n")
        for v in offset_utm:
            f.write(f"  - {format_float(v)}\n")


def remapid_and_reset_poses_offset(root_dir, output_dir, poses_folder, oldid2newid, center_utm=None):
    """
    遍历 poses_folder 下所有 pose yaml 文件，
    root_dir: 项目根目录
    poses_folder: poses 文件夹相对路径，如 'sparse/vehicle_geo_pose'
    oldid2newid: (old_id, time) -> new_id 映射
    """
    if center_utm is None:
        print("未指定 center_utm 使用默认 (0.0, 0.0)")
        center_utm = (0.0, 0.0)
        exit(1)

    # 构造原始 pose 文件夹路径
    poses_src_dir = os.path.join(root_dir, poses_folder)
    # 构造目标 pose 文件夹路径
    poses_dst_dir = os.path.join(output_dir, poses_folder)
    # 创建目标文件夹（递归创建）
    os.makedirs(poses_dst_dir, exist_ok=True)
    if not os.path.exists(poses_src_dir):
        print(f"poses 文件夹不存在: {poses_src_dir}")
        return []
    pose_files = [f for f in os.listdir(poses_src_dir) if f.endswith('.yaml') and os.path.isfile(os.path.join(poses_src_dir, f))]
    for fname in pose_files:
        yaml_path = os.path.join(poses_src_dir, fname)
        # print(f"读取 {yaml_path}")
        # 利用文件名解析出 id 和 time
        res = parse_id_time(fname)
        if not res:
            print(f"文件名无法解析: {fname}")
            continue
        old_id, t, cam, ext = res
        key = (old_id, t)
        if key not in oldid2newid:
            print(f"未找到映射: {key}")
            continue
        new_id = oldid2newid[key]
        t_str = f"{t:.3f}"
        if cam:
            new_name = f"{new_id}_{t_str}_{cam}{ext}"
        else:
            new_name = f"{new_id}_{t_str}{ext}"
        # print(f"新文件名: {new_name}")

        old_pose = load_lidar_pose_from_file(yaml_path)
        # 读取内容示例
        # old_pose = {
        #     'translation': translation,
        #     'rotation_matrix': rotation_matrix,
        #     'quaternion': quaternion,
        #     'timestamp': timestamp,
        #     'pose_utm': pose_utm_matrix,
        #     'offset_utm': np.array(offset_utm)
        # }
        new_pose_file = os.path.join(poses_dst_dir, new_name)
        # print(f"写入  {new_pose_file}")

        offset_utm = np.array([center_utm[0], center_utm[1], 0.0], dtype=np.float64)

        pose_utm = old_pose['pose_utm'].copy()
        # 旋转部分保持不变
        pose_utm[:3, :3] =  old_pose['rotation_matrix']
        # 平移部分减去offset_utm
        pose_utm[:3, 3] = old_pose['translation'] - offset_utm
        # print(f"new pose_utm: {pose_utm}")

        write_pose_to_yaml(new_pose_file,
            pose_utm,
            offset_utm,
            old_pose['timestamp'] )

def check_image_is_black(img_path):
    from PIL import Image
    is_black = False
    try:
        if os.path.exists(img_path):
            with Image.open(img_path) as im:
                # 先缩小图片，加快全黑检测
                im_small = im.resize((32, 32), Image.NEAREST)
                im_arr = np.array(im_small)
                # 支持灰度和RGB
                if im_arr.ndim == 2:
                    is_black = np.all(im_arr == 0)
                elif im_arr.ndim == 3:
                    is_black = np.all(im_arr == 0)
                else:
                    is_black = False
                if is_black:
                    print(f"[INFO] 跳过全黑图片: {img_path}")
        else:
            print(f"[WARN] 图片不存在: {img_path}")
    except Exception as e:
        print(f"[ERROR] 检查图片是否全黑失败: {img_path}, err={e}")
    
    return is_black

def remap_and_copy(src_dir, dst_dir, oldid2newid):
    files = [f for f in os.listdir(src_dir) if os.path.isfile(os.path.join(src_dir, f))]
    count = 0
    cnt_black_images = 0
    for f in files:
        res = parse_id_time(f)
        if not res:
            continue
        old_id, t, cam, ext = res
        key = (old_id, t)
        if key not in oldid2newid:
            continue
        new_id = oldid2newid[key]
        t_str = f"{t:.3f}"
        if cam:
            new_name = f"{new_id}_{t_str}_{cam}{ext}"
        else:
            new_name = f"{new_id}_{t_str}{ext}"
        # img_path = os.path.join(src_dir, f)
        # # 文件名含有images字符才检查是否全黑
        # if '/images/' in img_path.lower():
        #     # print(f"  img_path: {img_path}")
        #     if check_image_is_black(img_path):
        #         cnt_black_images += 1
        #         continue
                
        shutil.copy2(os.path.join(src_dir, f), os.path.join(dst_dir, new_name))
        count += 1
    # print(f"  {src_dir}: {count} files (remap id)")
    # print(f"cnt_black_images: {cnt_black_images} files (skipped)")
    return count

def main(root_dir, output_directory="../resorted_data/", center_utm=None):
    # 需要递归处理的子文件夹
    remapid_folders = ["images/cam1","images/cam2","images/cam3",
    "images/cam4","images/cam5","images/cam6","images/cam7",
    "masks/dynamic/cam1","masks/dynamic/cam2","masks/dynamic/cam3",
    "masks/dynamic/cam4","masks/dynamic/cam5","masks/dynamic/cam6",
    "masks/dynamic/cam7",
    "masks/sky/cam1","masks/sky/cam2","masks/sky/cam3",
    "masks/sky/cam4","masks/sky/cam5","masks/sky/cam6",
    "masks/sky/cam7",
    "odoms", "labels", "pointclouds", "debug_files/3dbox"
    ]

    poses_folders = "sparse/vehicle_geo_pose"
    passthrough_folders = ["calib", "hd_map", "masks/static"]

    debug_dir = os.path.join(output_directory)
    os.makedirs(debug_dir, exist_ok=True)

    os.makedirs(os.path.join(output_directory, "masks/static"), exist_ok=True)

    # 1. 以odoms为准，建立 old_id -> new_id 映射
    odoms_dir = os.path.join(root_dir, "odoms")
    odom_files = [f for f in os.listdir(odoms_dir) if os.path.isfile(os.path.join(odoms_dir, f))]
    odom_id_time = []
    for f in odom_files:
        res = parse_id_time(f)
        if res:
            id_, t, cam, ext = res
            odom_id_time.append((id_, t, f))
    # 按time排序
    odom_id_time.sort(key=lambda x: x[1])
    # 新id分配，严格从0递增，且不重复，按(原id, time)唯一
    oldid_time2newid = {}
    new_id = 0
    for item in odom_id_time:
        old_id, t, fname = item
        key = (old_id, t)
        if key not in oldid_time2newid:
            oldid_time2newid[key] = new_id
            new_id += 1
    # print(f"odoms (id,time)->new_id映射: {oldid_time2newid}")
    
    # vehicle_geo_pose 也需要重命名 并且将其offset_utm重置 到 指定的位置为原点
    remapid_and_reset_poses_offset(root_dir, output_directory , poses_folders, oldid_time2newid, center_utm)

    for sub in remapid_folders + passthrough_folders:
        src = os.path.join(root_dir, sub)
        if not os.path.exists(src):
            continue
        dst = os.path.join(debug_dir, sub)
        if sub in passthrough_folders:
            # 确保目标父目录存在
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            try:
                if os.path.isdir(src):
                    if os.path.exists(dst):
                        shutil.rmtree(dst)
                    shutil.copytree(src, dst)
                    print(f"Passthrough copied {src} -> {dst}")
                else:
                    # 确保父目录存在
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(src, dst)
                    print(f"Passthrough copied file {src} -> {dst}")
            except Exception as e:
                print(f"Passthrough copy failed for {src} -> {dst}: {e}")
            continue
        os.makedirs(dst, exist_ok=True)
        # 只对remapid_folders用id映射
        if sub in remapid_folders:
            remap_and_copy(src, dst, oldid_time2newid)
        else:
            count = process_folder(src, dst)
            print(f"Processed {src} -> {dst}, files: {count}")

if __name__ == "__main__":
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='重命名数据按时间排序')
    parser.add_argument('--longitude', '--lon', type=float, default=None,
                        help='Target longitude (default: from config)')
    parser.add_argument('--latitude', '--lat', type=float, default=None,
                        help='Target latitude (default: from config)')
    parser.add_argument('--roi_distance_min', type=float, default=None,
                        help='ROI distance min in meters (default: from config)')
    parser.add_argument('--roi_distance_max', type=float, default=None,
                        help='ROI distance max in meters (default: from config)')
    args = parser.parse_args()
    
    # 加载配置
    config = load_config()
    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    # 搜索参数 复用 search_params 中的默认位置 - 命令行参数优先
    target_lat = args.latitude if args.latitude is not None else default_location.get('latitude', 31.41033324)
    target_lon = args.longitude if args.longitude is not None else default_location.get('longitude', 120.65582103)
    roi_distance_max = args.roi_distance_max if args.roi_distance_max is not None else search_params.get('roi_distance_max', 100.0)
    roi_distance_min = args.roi_distance_min if args.roi_distance_min is not None else search_params.get('roi_distance_min', 50.0)
    print(f"中心点坐标: ({target_lat}, {target_lon})")
    x, y, epsg_code, hemisphere = lonlat_to_utm(target_lon, target_lat)
    center_utm = (x, y)
    print(f"中心点UTM坐标: ({x}, {y}), EPSG: {epsg_code}, Hemisphere: {hemisphere}")
    # exit()

    # 复用
    # 输出目录和会话名模板直接从 search_params 读取
    output_root = Path(search_params.get('export_dir', '/mnt/nvme0n2/project/postgresql/export/'))
    session_name_template = search_params.get('session_name_template', 'search_{lat:.6f}_{lon:.6f}_{roi_distance_min}_{roi_distance_max}m')
    session_name = session_name_template.format(lat=target_lat, lon=target_lon, roi_distance_min=roi_distance_min, roi_distance_max=roi_distance_max)
    default_project_dir = output_root / session_name
    print(f"默认项目目录: {default_project_dir}")

    output_3dgs_data = Path(search_params.get('3dgs_dir', '/data/3dgs_data/'))
    print(f"重命名输出路径: {output_3dgs_data}")
    output_directory = output_3dgs_data / (session_name + "_resorted")
    print(f"重命名输出路径: {output_directory}")
    
    main(default_project_dir, output_directory=output_directory, center_utm=center_utm)
