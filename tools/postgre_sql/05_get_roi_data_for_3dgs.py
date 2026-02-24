#!/usr/bin/env python3

import numpy as np
import os
from pathlib import Path
import argparse

# 新增: 导入PIL用于图片处理
from PIL import Image

# 导入配置加载函数
from qt.load_config import load_config

# 导入传感器参数管理器
from qt.sensorparam import SensorExtrinsicsManager, CameraIntrinsicsManager

# 导入3D box投影工具函数
from qt.box_projection import (
    calculate_3dbox_projection_ratio,

    load_lidar_pose_from_file,
    plot_camera_positions_simple,
    lonlat_to_utm,
    utm_to_lonlat
)

# 记录本次程序运行中，哪些输出文件已经“首次写入过”
_OUTPUT_FILE_INITIALIZED = set()

def write_image_path_to_file(img_path, output_file_path):
    """将图像路径写入指定文件
    
    Args:
        img_path: 图像的绝对路径
        output_file_path: 输出文件路径
    """
    try:
        output_path = Path(output_file_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # 同一次运行：第一次写入时清空旧内容，后续追加
        output_key = str(output_path)
        mode = 'a' if output_key in _OUTPUT_FILE_INITIALIZED else 'w'
        _OUTPUT_FILE_INITIALIZED.add(output_key)

        with open(output_path, mode, encoding='utf-8') as f:
            f.write(f"{img_path}\n")
    except Exception as e:
        print(f"[ERROR] 写入路径失败: {e}")

import subprocess
import threading
def run_cpp_task():
    print("[INFO] 启动06_nu_sampling子进程...")
    result = subprocess.run(["./build/06_nu_sampling", "-i", str(work_dir)])
    if result.returncode != 0:
        print(f"[ERROR] ./build/06_nu_sampling 执行失败，返回码: {result.returncode}")
    else:
        print("./build/06_nu_sampling 执行成功")


if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Get ROI data for 3DGS')
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
    
    # 从配置文件获取经纬度和ROI参数
    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    
    # 命令行参数优先，否则使用配置文件的值
    target_lon = args.longitude if args.longitude is not None else default_location.get('longitude', 120.65582103)
    target_lat = args.latitude if args.latitude is not None else default_location.get('latitude', 31.41033324)
    roi_distance_min = args.roi_distance_min if args.roi_distance_min is not None else search_params.get('roi_distance_min', 100.0)  # XY 前后方向小于这个值的直接保留
    roi_distance_max = args.roi_distance_max if args.roi_distance_max is not None else search_params.get('roi_distance_max', 200.0)  # XY 前后方向大于这个值的直接丢弃

    if roi_distance_min >= roi_distance_max:
        print(f"[ERROR] roi_distance_min ({roi_distance_min}) 必须小于 roi_distance_max ({roi_distance_max})")
        exit(1)

    search_params = config.get('search_params', {})
    # 输出目录和会话名模板直接从 search_params 读取
    output_root = Path(search_params.get('export_dir', '/mnt/nvme0n2/project/postgresql/export/'))
    session_name_template = search_params.get('session_name_template', 'search_{lat:.6f}_{lon:.6f}_{roi_distance_min}_{roi_distance_max}m')
    session_name = session_name_template.format(lat=target_lat, lon=target_lon, roi_distance_min=roi_distance_min, roi_distance_max=roi_distance_max)
    default_project_dir = output_root / session_name

    # done 0130
    # 配置文件添加 3dgs_dir 参数， 输出的目录改为 /data/3dgs_data/search_xxx_resorted
    dir_3dgs_data = Path(search_params.get('3dgs_dir', '/data/3dgs_data/'))

    # Done 0128
    # todo 调试用路径
    # 整个流程重新 组织
    work_dir = dir_3dgs_data / (session_name + "_resorted/")
    if not work_dir.exists():
        print(f"[ERROR] 输入目录不存在: {work_dir}")
        exit(1)
    print(f"输入目录 : {work_dir}")
    # exit(0)

    # 启动子线程运行06_nu_sampling
    cpp_thread = threading.Thread(target=run_cpp_task, daemon=True)
    cpp_thread.start()
    print("[INFO] 主线程继续执行 等待06_nu_sampling结束 才会结束。")


    # 投影占比阈值
    projected_ratio_threshold = search_params.get('projected_ratio_threshold', 0.2)  

    # 从 配置获取输出路径文件
    file_for_3dgs_name = search_params.get('file_for_3dgs_name', 'img_whitelist_for_3dgs.txt')
    file_for_3dgs_paths = work_dir / file_for_3dgs_name
    
    print(f"file_for_3dgs_paths = {file_for_3dgs_paths}")
    print(f"projected_ratio_threshold = {projected_ratio_threshold}")
    print(f"roi_distance_min = {roi_distance_min}")
    print(f"roi_distance_max = {roi_distance_max}")
    # exit()

    intrinsics_dir = work_dir / "calib/new_intrinsics"
    extrinsics_dir = work_dir / "calib/extrinsics"
    pose_dir = work_dir / "sparse/vehicle_geo_pose"

    # 转换为UTM坐标
    utm_x, utm_y, zone, hemisphere = lonlat_to_utm(target_lon, target_lat)
    print(f"输入经纬度: ({target_lon}, {target_lat})")
    print(f"UTM分区: {zone}{hemisphere}")
    print(f"UTM坐标: X={utm_x:.2f}m, Y={utm_y:.2f}m")

    box_center_map_demo = np.array([0.0, 0.0, 0.0])  # 中心点 (x, y, z)
    box_size_lwh_demo = np.array([roi_distance_min, roi_distance_min, 50.0])     # 长宽高 (length, width, height)
    
    # 3D box的旋转（绕z轴旋转45度）
    theta = np.radians(0.0)
    box_rotation_matrix_demo = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])


    if os.path.exists(intrinsics_dir) and os.path.exists(extrinsics_dir):
        # 1. 加载相机内参
        print("\n[步骤1] 加载相机内参...")
        cam_intrinsics_mgr = CameraIntrinsicsManager(intrinsics_dir)
        print(f"  加载了 {len(cam_intrinsics_mgr)} 个相机内参")
        
        # 2. 加载传感器外参
        print("\n[步骤2] 加载传感器外参...")
        sensor_extrinsics_mgr = SensorExtrinsicsManager(extrinsics_dir)
        print(f"  加载了 {len(sensor_extrinsics_mgr)} 个传感器外参")

    # 遍历pose_dir下的所有yaml文件
    print("\n" + "="*50)
    print("读取所有雷达pose文件")
    print("="*50)
    
    lidar_poses = []
    if pose_dir.exists() and pose_dir.is_dir():
        # 获取所有yaml文件，按文件名中的ID排序
        yaml_files = list(pose_dir.glob("*.yaml"))
        # 文件名格式: {id}_{timestamp}.yaml，按ID排序
        yaml_files = sorted(yaml_files, key=lambda f: int(f.stem.split('_')[0]))
        
        # 存储所有pose数据
        for yaml_file in yaml_files:
            # print(f"\n处理文件: {yaml_file.name}")
            # utm坐标系下的雷达pose 没有offset
            lidar_pose_map = load_lidar_pose_from_file(str(yaml_file))
            if lidar_pose_map is not None:
                # 以utm_x, utm_y为offset，减去偏移量
                lidar_pose_map['translation'][0] -= utm_x
                lidar_pose_map['translation'][1] -= utm_y
                lidar_poses.append({
                    'filename': yaml_file.name,
                    'pose': lidar_pose_map
                })
                # print(f"  ✓ 位置: {lidar_pose_map['translation']}")
                # if 'quaternion' in lidar_pose_map:
                #     print(f"  ✓ 四元数: {lidar_pose_map['quaternion']}")
        print(f"\n成功读取 {len(lidar_poses)} 个pose")
    else:
        print(f"[ERROR] 目录不存在: {pose_dir}")
        lidar_poses = []
    

    # 处理多个相机（cam1到cam7）
    print("\n" + "="*50)
    print("计算所有相机在map坐标系下的pose")
    print("="*50)
    
    frame_id = 'hesai128'   # 'vehicle'  # sparse数据中的实际是 雷达 在map坐标系下的pose
    all_cameras_poses = {}  # 存储所有相机的poses
    
    # 初始化所有相机的pose列表
    for cam_num in range(1, 8):
        child_frame_id = f'cam{cam_num}'
        all_cameras_poses[child_frame_id] = []
    
    # 遍历所有lidar poses
    cnt_roi_used = 0
    cnt_roi_within_min = 0

    cnt = 0
    cnt_black_images = 0  # 新增: 统计全黑图片数量
    for lidar_pose_data in lidar_poses:
        # cnt += 1
        # if cnt % 10 == 0:
        #     pass
        # else:
        #     continue
        
        # 深拷贝pose数据，避免修改原始数据
        lidar_pose = {
            'translation': lidar_pose_data['pose']['translation'].copy(),
            'rotation_matrix': lidar_pose_data['pose']['rotation_matrix'].copy()
        }
        
        # 计算lidar位置距离原点的距离（用于判断是否需要处理）
        x = abs(lidar_pose['translation'][0])
        y = abs(lidar_pose['translation'][1])

        # 构建vehicle在map坐标系的变换矩阵
        T_vehicle_to_map = np.eye(4)
        T_vehicle_to_map[:3, :3] = lidar_pose['rotation_matrix']
        T_vehicle_to_map[:3, 3] = lidar_pose['translation']
        
        # 遍历cam1到cam7
        for cam_num in range(1, 8):
            child_frame_id = f'cam{cam_num}'

            # 获取该相机到vehicle的变换
            T_cam_to_vehicle = sensor_extrinsics_mgr.get_transform(child_frame_id, frame_id)
            # print(f"  T_cam_to_vehicle for {child_frame_id}:\n{T_cam_to_vehicle}")

            # 相机内参
            cam_intr_demo = cam_intrinsics_mgr.get_intrinsics(child_frame_id)
            # print(f"  相机内参 for {child_frame_id}:\n{cam_intr_demo}")

            # 计算相机在map坐标系下的变换
            T_cam_to_map = T_vehicle_to_map @ T_cam_to_vehicle
            cam_pose_map = {
                'translation': T_cam_to_map[:3, 3].copy(),  # 使用copy()避免引用问题
                'rotation_matrix': T_cam_to_map[:3, :3].copy()  # 使用copy()避免引用问题
            }

            # 根据距离原点的距离决定是否保存pose
            should_add = False
            img_relative_path =  Path( child_frame_id ) / f"{lidar_pose_data['filename'].replace('.yaml', f'_{child_frame_id}.jpg')}" 
            img_path = str(work_dir / Path("images") / img_relative_path)
            pose_path = str(pose_dir  /  lidar_pose_data['filename'])
            # img_path = str("/data/dwm_data/park_20251120_0/images/cam7/863_1763621416.700_cam7.jpg")
            # print(f"  pose_path: {pose_path}")

            # 检查图片是否全黑，如果全黑则跳过
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
                            cnt_black_images += 1
                            print(f"[INFO] 跳过全黑图片: {img_path}")
                            continue
                else:
                    print(f"[WARN] 图片不存在: {img_path}")
                    continue
            except Exception as e:
                print(f"[ERROR] 检查图片是否全黑失败: {img_path}, err={e}")
                continue

            debug = False
            # debug = True
            if debug:
                # 保存debug图像的临时目录
                temp_dir = Path("/data/postgresql/geo-database-engine/data/debug_temp/")
                try:
                    temp_dir.mkdir(parents=True, exist_ok=True)
                except Exception as e:
                    print(f"[ERROR] 创建debug_temp目录失败: {temp_dir}, err={e}")
                output_img_path = str(temp_dir / f"{lidar_pose_data['filename'].replace('.yaml', f'_test_{child_frame_id}.jpg')}")
            else:
                output_img_path = None

            # print(f"  图像: {img_path}")
            # print(f"  输出: {output_img_path}")

            # should_add = True
            if x < roi_distance_min and y < roi_distance_min:
                should_add = True
                cnt_roi_within_min = cnt_roi_within_min + 1
            elif x > roi_distance_max or y > roi_distance_max:
                should_add = False
            else:
                # 情况3: 介于100到200米之间，调用投影函数判断
                # 只考虑了XY矩阵方向的距离，最大距离 是 sqrt(2) * roi_distance_max
                ratio_demo = calculate_3dbox_projection_ratio(
                    box_center=box_center_map_demo,
                    box_size=box_size_lwh_demo,
                    box_rotation=box_rotation_matrix_demo,
                    camera_translation=cam_pose_map['translation'],
                    camera_rotation=cam_pose_map['rotation_matrix'],
                    camera_intrinsics=cam_intr_demo,
                    background_image=img_path,
                    output_image=output_img_path,
                    image_width=cam_intr_demo['image_width'],
                    image_height=cam_intr_demo['image_height']
                )
                # print(f"  {lidar_pose_data['filename']}: 3D Box在{child_frame_id}中的投影占比: {ratio_demo:.4%}")
                if ratio_demo < projected_ratio_threshold:
                    should_add = False
                else:
                    cnt_roi_used = cnt_roi_used + 1
                    should_add = True

            if should_add:
                # 写入 pose 和 img 路径到文件
                write_image_path_to_file(img_relative_path, file_for_3dgs_paths)
                # write_image_path_to_file(pose_path, file_for_3dgs_paths)
                all_cameras_poses[child_frame_id].append({
                    'filename': lidar_pose_data['filename'],
                    'pose': cam_pose_map
                })
    
    # 打印每个相机的统计信息
    print(f"\n成功处理 {len(lidar_poses)} 个lidar pose")
    for cam_num in range(1, 8):
        child_frame_id = f'cam{cam_num}'
        print(f"  {child_frame_id}: {len(all_cameras_poses[child_frame_id])} 个pose")
    
    # 绘制所有相机的轨迹和朝向
    print("\n" + "="*50)
    print("绘制所有相机轨迹")
    print("="*50)
    
    output_path_simple = "./data/cameras_pose.png"
    plot_camera_positions_simple(
        box_center=box_center_map_demo,
        box_size=box_size_lwh_demo,
        cam_poses_in_map=all_cameras_poses, 
        save_path=str(output_path_simple), 
        point_size=1
    )
    # 拷贝图片到输出目录
    try:
        output_img_dst = Path(work_dir) / "cameras_pose.png"
        os.makedirs(output_img_dst.parent, exist_ok=True)
        import shutil
        shutil.copy2(output_path_simple, output_img_dst)
        print(f"cameras_pose.png copied to {output_img_dst}")
    except Exception as e:
        print(f"[ERROR] 拷贝cameras_pose.png失败: {e}")

    print(f"file_for_3dgs_paths = {file_for_3dgs_paths}")
    print(f"cnt_roi_used at min to max = {cnt_roi_used}")
    print(f"cnt_roi_within_min :{cnt_roi_within_min}")
    print(f"跳过全黑图片数量: {cnt_black_images}")

    # 如需等待子线程结束，可加:
    cpp_thread.join()
    
    exit()
    