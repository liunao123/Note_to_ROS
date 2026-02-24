#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from glob import glob
import os
import sys
import argparse
import numpy as np
import yaml
import json
import pandas as pd
from pathlib import Path
from pyntcloud import PyntCloud
from scipy.spatial import cKDTree


# 定义全局offset
offset_map = np.array([274402.850498005573 , 3479086.707035151776, 0.0])
offset_map_initialized = False  # 标记offset_map是否已初始化

# 导入3D box投影工具函数
from qt.box_projection import (
    load_lidar_pose_from_file,
    get_all_session_dirs,
    # calculate_3dbox_projection_ratio,
    # plot_camera_positions_simple,
    lonlat_to_utm,
    # utm_to_lonlat
)


def load_ply(ply_path):
    """
    加载PLY点云文件
    Args:
        ply_path: PLY文件路径
    Returns:
        PyntCloud对象
    """
    cloud = PyntCloud.from_file(ply_path)
    print(f"Loaded PLY from {ply_path}, point count: {len(cloud.points)}")
    return cloud


def crop_points_by_xy(cloud, x_min=-300, x_max=300, y_min=-300, y_max=300, z_min=-300, z_max=300):
    """
    按XY范围裁剪点云
    Args:
        cloud: 点云，可以是PyntCloud对象或DataFrame
        x_min, x_max, y_min, y_max: XY范围
    Returns:
        裁剪后的点云，类型与输入一致
    """
    # 判断输入类型
    is_pyntcloud = isinstance(cloud, PyntCloud)
    
    # 获取DataFrame
    df = cloud.points if is_pyntcloud else cloud
    
    # print(f"Original point count: {len(df)}")
    print(f"XY范围: x=[{df['x'].min():.2f}, {df['x'].max():.2f}], y=[{df['y'].min():.2f}, {df['y'].max():.2f}]")
    print(f"裁剪范围: x=[{x_min}, {x_max}], y=[{y_min}, {y_max}]")
    
    mask = (
        (df['x'] >= x_min) & (df['x'] <= x_max) &
        (df['y'] >= y_min) & (df['y'] <= y_max)
    )
    cropped_df = df[mask]
    print(f"Cropped point count: {len(cropped_df)}")
    
    # 返回与输入相同的类型
    if is_pyntcloud:
        return PyntCloud(cropped_df)
    else:
        return cropped_df


def crop_points_by_dis(cloud, max_distance=300):
    """
    按距离原点的距离裁剪点云，删除距离大于max_distance的点
    Args:
        cloud: 点云，可以是PyntCloud对象或DataFrame
        max_distance: 最大距离（米），默认300m
    Returns:
        裁剪后的点云，类型与输入一致
    """
    # 判断输入类型
    is_pyntcloud = isinstance(cloud, PyntCloud)
    
    # 获取DataFrame
    df = cloud.points if is_pyntcloud else cloud
    
    # print(f"Original point count: {len(df)}")
    
    # 计算每个点到原点的距离
    distances = np.sqrt(df['x']**2 + df['y']**2 + df['z']**2)
    print(f"距离范围: [{distances.min():.2f}m, {distances.max():.2f}m]")
    print(f"裁剪阈值: {max_distance}m")
    
    # 保留距离小于等于max_distance的点
    mask = distances <= max_distance
    cropped_df = df[mask]
    
    removed_count = len(df) - len(cropped_df)
    print(f"删除了 {removed_count} 个点（距离>{max_distance}m）")
    print(f"Cropped point count: {len(cropped_df)}")
    
    # 返回与输入相同的类型
    if is_pyntcloud:
        return PyntCloud(cropped_df)
    else:
        return cropped_df


def save_ply(df, output_path):
    """
    保存点云DataFrame为PLY文件
    Args:
        df: 点云DataFrame
        output_path: 输出文件路径
    """
    if df is None:
        print("Warning: merged_cloud is None, skipping save for no-sky version")
        return
    cloud = PyntCloud(df)
    cloud.to_file(output_path)
    print(f"PLY saved to {output_path}")

     
def get_first_yaml_from_vehicle_pose(work_dir):
    """
    从vehicle_geo_pose目录获取第一个yaml文件的绝对路径
    Args:
        work_dir: 工作目录路径
    Returns:
        第一个yaml文件的绝对路径，如果没有找到则返回None
    """
    pose_dir = os.path.join(work_dir, "sparse/vehicle_geo_pose")
    if not os.path.exists(pose_dir):
        print(f"Warning: pose directory not found: {pose_dir}")
        return None
    
    yaml_files = glob(os.path.join(pose_dir, "*.yaml"))
    if not yaml_files:
        print(f"Warning: no yaml files found in {pose_dir}")
        return None
    
    yaml_files.sort()
    return yaml_files[0]


def load_pose_and_transform(pose_path):
    """
    读取位姿文件并转换到相对坐标系
    Args:
        pose_path: 位姿文件路径
    Returns:
        转换后的pose (numpy array)
    """
    
    pose = load_lidar_pose_from_file(pose_path)["offset_utm"]
    print("offset_utm:\n", pose)
    
    global offset_map, offset_map_initialized
    # 第一次调用时，用读取到的值初始化offset_map
    if not offset_map_initialized:
        offset_map = np.array(pose)
        offset_map_initialized = True
        print("==========================")
        print("首次初始化offset_map:\n", offset_map)
        print("==========================")
    
    print("==========================")
    print("减去全局offset:\n", offset_map)
    print("==========================")
    # 减去全局offset
    offset_map_reshaped = offset_map.reshape(1, 3)
    pose = np.array(pose) - offset_map_reshaped
    print("转换后的pose:\n", pose)
    return pose


def gs_ply_to_world_temp(cloud, pose):
    """
    将点云转换到world坐标系下
    Args:
        cloud: PyntCloud对象
        pose: 位姿numpy array (1, 3) 或 (3,)
    Returns:
        转换后的PyntCloud对象
    """
    df = cloud.points.copy()
    print(f"原始点数: {len(df)}")
    
    # 只处理x,y,z
    pts = df[['x', 'y', 'z']].values
    pts_world = pts + pose.reshape(1, 3)
    # 更新df
    df[['x', 'y', 'z']] = pts_world
    
    world_cloud = PyntCloud(df)
    print(f"转换完成，点数: {len(df)}")
    return world_cloud


def strategy_best_of_both(data1, data2, props, threshold=0.01):
    """策略5: 择优融合 - 对每个空间位置保留质量更好的高斯"""
    print(f"\n📊 使用策略207 : 择优融合（空间阈值={threshold}）")
    
    import time
    t0 = time.time()
    xyz1 = np.stack([data1['x'], data1['y'], data1['z']], axis=1)
    xyz2 = np.stack([data2['x'], data2['y'], data2['z']], axis=1)
    t1 = time.time()

    # 计算两个点云的外接框
    min1 = xyz1.min(axis=0)
    max1 = xyz1.max(axis=0)
    min2 = xyz2.min(axis=0)
    max2 = xyz2.max(axis=0)
    t2 = time.time()

    # 计算交集区域，并适当扩大
    margin = 5.0 # 可调节，扩大交集区域
    intersect_min = np.maximum(min1, min2) - margin
    intersect_max = np.minimum(max1, max2) + margin

    # 判断哪些点在交集区域
    def in_box(xyz, box_min, box_max):
        return np.all((xyz >= box_min) & (xyz <= box_max), axis=1)

    mask1 = in_box(xyz1, intersect_min, intersect_max)
    mask2 = in_box(xyz2, intersect_min, intersect_max)
    t3 = time.time()

    # 只对交集区域做去重，其他区域直接保留
    xyz1_in = xyz1[mask1]
    xyz2_in = xyz2[mask2]
    t4 = time.time()

    # 计算质量指标（不透明度 × 平均缩放的倒数 = 更不透明且更小的高斯更好）
    opacity1 = 1 / (1 + np.exp(-data1['opacity']))
    opacity2 = 1 / (1 + np.exp(-data2['opacity']))
    scale1 = np.exp(np.stack([data1['scale_0'], data1['scale_1'], data1['scale_2']], axis=1))
    scale2 = np.exp(np.stack([data2['scale_0'], data2['scale_1'], data2['scale_2']], axis=1))
    quality1 = opacity1 * (1.0 / (scale1.mean(axis=1) + 1e-6))
    quality2 = opacity2 * (1.0 / (scale2.mean(axis=1) + 1e-6))
    t5 = time.time()

    # 只对交集区域做KD树去重
    t_kd_start = time.time()
    tree1 = cKDTree(xyz1_in, leafsize=32)
    distances, indices = tree1.query(xyz2_in)
    duplicate_mask = distances < threshold
    t_kd_end = time.time()
    num_duplicates = duplicate_mask.sum()
    print(f"   交集区域: {intersect_min} ~ {intersect_max}")
    print(f"   交集区域内: 模型1 {len(xyz1_in)}，模型2 {len(xyz2_in)}")
    print(f"   检测到 {num_duplicates:,} 对接近点")

    # 初始化保留mask
    keep_from_model1 = np.ones(len(xyz1), dtype=bool)
    keep_from_model2 = np.ones(len(xyz2), dtype=bool)

    # 交集区域外的点全部保留
    # 交集区域内的点做去重
    # 需要找到交集区域内的点在原数组中的索引
    idx1_in = np.where(mask1)[0]
    idx2_in = np.where(mask2)[0]

    t_loop_start = time.time()
    replaced_count = 0
    for i_in, is_dup in enumerate(duplicate_mask):
        if not is_dup:
            continue
        i2 = idx2_in[i_in]
        i1 = idx1_in[indices[i_in]]
        if quality2[i2] > quality1[i1]:
            keep_from_model1[i1] = False
            replaced_count += 1
        else:
            keep_from_model2[i2] = False
    t_loop_end = time.time()

    print(f"   用模型2替换了 {replaced_count:,} 个点")
    print(f"   保留模型1: {keep_from_model1.sum():,} 个点")
    print(f"   保留模型2: {keep_from_model2.sum():,} 个点")

    # 合并
    t_merge_start = time.time()
    merged = {}
    for prop in props:
        prop_name = prop.name
        merged[prop_name] = np.concatenate([
            data1[prop_name][keep_from_model1],
            data2[prop_name][keep_from_model2]
        ])
    t_merge_end = time.time()

    print(f"   融合后总点数: {len(merged['x']):,}")
    print(f"   计时: ")
    print(f"      stack/box: {t1-t0:.3f}s, bbox: {t2-t1:.3f}s, mask: {t3-t2:.3f}s, xyz_in: {t4-t3:.3f}s, quality: {t5-t4:.3f}s")
    print(f"      KDTree: {t_kd_end-t_kd_start:.3f}s, loop: {t_loop_end-t_loop_start:.3f}s, merge: {t_merge_end-t_merge_start:.3f}s")
    print(f"      总计: {t_merge_end-t0:.3f}s")
    return merged

def merge_ply_strategy_simple(cloud1, cloud2):
    """策略1: 简单合并 - 直接连接两个点云
    Args:
        cloud1: 第一个点云，PyntCloud对象或DataFrame
        cloud2: 第二个点云，PyntCloud对象或DataFrame
    Returns:
        合并后的PyntCloud对象
    """
    print("\n📊 使用策略: 简单合并（直接连接）")
    
    # 判断输入类型并获取DataFrame
    df1 = cloud1.points if isinstance(cloud1, PyntCloud) else cloud1
    df2 = cloud2.points if isinstance(cloud2, PyntCloud) else cloud2
    
    print(f"   模型1点数: {len(df1):,}")
    print(f"   模型2点数: {len(df2):,}")
    
    # 使用pandas的concat进行拼接（比append更推荐）
    merged_df = pd.concat([df1, df2], ignore_index=True)
    
    print(f"   融合后: {len(merged_df):,}")
    
    return PyntCloud(merged_df)

def merge_ply(cloud_list, threshold=0.025, use_best_of_both=False):
    """
    合并多个PyntCloud对象
    Args:
        cloud_list: PyntCloud对象列表
        threshold: 空间距离阈值，用于判断是否为同一位置的高斯（仅在use_best_of_both=True时使用）
        use_best_of_both: 是否使用择优融合策略，False则使用简单拼接
    Returns:
        合并后的PyntCloud对象
    """
    if not cloud_list:
        print("Warning: empty cloud list")
        return None
    
    if len(cloud_list) == 1:
        return cloud_list[0]
    
    if use_best_of_both:
        # 使用择优融合策略
        print(f"合并 {len(cloud_list)} 个点云，使用择优融合策略...")
        
        # 将第一个点云转换为字典格式
        df1 = cloud_list[0].points
        merged_data = {col: df1[col].values for col in df1.columns}
        
        # 获取属性列表（用于strategy_best_of_both）
        class PropMock:
            def __init__(self, name):
                self.name = name
        props = [PropMock(col) for col in df1.columns]
        
        # 逐个融合后续的点云
        for i in range(1, len(cloud_list)):
            print(f"\n正在融合第 {i+1}/{len(cloud_list)} 个点云...")
            df2 = cloud_list[i].points
            data2 = {col: df2[col].values for col in df2.columns}
            
            # 使用择优融合策略
            merged_data = strategy_best_of_both(merged_data, data2, props, threshold=threshold)
        
        # 将字典转换回DataFrame
        merged_df = pd.DataFrame(merged_data)
        merged_cloud = PyntCloud(merged_df)
        
        print(f"\n✅ 合并完成，最终点数: {len(merged_df):,}")
    else:
        # 使用简单拼接策略
        print(f"合并 {len(cloud_list)} 个点云，使用简单拼接策略...")
        df_list = [cloud.points for cloud in cloud_list]
        merged_df = df_list[0]
        
        for i in range(1, len(df_list)):
            merged_df = merged_df.append(df_list[i], ignore_index=True)
        
        merged_cloud = PyntCloud(merged_df)
        print(f"合并完成，总点数: {len(merged_df):,}")
    
    return merged_cloud


def merge_two_ply(ply_path1, ply_path2, output_ply_path):
    """
    合并两个PLY点云文件并保存
    Args:
        ply_path1: 第一个PLY文件路径
        ply_path2: 第二个PLY文件路径
        output_ply_path: 输出合并后的PLY文件路径
    """
    print(f"合并点云:\n  {ply_path1}\n  {ply_path2}\n到\n  {output_ply_path}")
    cloud1 = PyntCloud.from_file(ply_path1)
    cloud2 = PyntCloud.from_file(ply_path2)
    df1 = cloud1.points
    df2 = cloud2.points
    merged_df = df1.append(df2, ignore_index=True)
    merged_cloud = PyntCloud(merged_df)
    merged_cloud.to_file(output_ply_path)
    print(f"合并后点云已保存: {output_ply_path}, 点数: {len(merged_df)}")


def gs_ply_to_world(ply_path, pose_path, output_ply_path):
    """
    将PLY点云转换到world坐标系下，保存为新的PLY文件
    Args:
        ply_path: 输入PLY文件路径
        pose_path: 位姿文件路径（用load_lidar_pose_from_file读取）
        output_ply_path: 输出PLY文件路径
    """
    # 读取点云
    cloud = PyntCloud.from_file(ply_path)
    df = cloud.points
    print(f"原始点数: {len(df)}")

    # 读取并转换位姿
    pose = load_pose_and_transform(pose_path)

    # 只处理x,y,z
    pts = df[['x', 'y', 'z']].values
    pts_world = pts + pose.reshape(1, 3)
    # 更新df
    df[['x', 'y', 'z']] = pts_world
    cloud_world = PyntCloud(df)
    cloud_world.to_file(output_ply_path)
    print(f"转换后点云已保存: {output_ply_path}, 点数: {len(df)}")

# todo  看一下为什么读取 sky.ply失败了
from plyfile import PlyData, PlyElement

def sky_cloud_is_big_enough(cloud, sky_cloud):
    """
    确保sky_cloud能够完全包含cloud点云
    如果cloud的外接box大于sky_cloud，则将sky_cloud的Z>0部分同比例扩大
    Args:
        cloud: 主点云（PyntCloud对象或DataFrame）
        sky_cloud: 天空点云（PyntCloud对象或DataFrame）
    Returns:
        调整后并合并的点云（PyntCloud对象）
    """
    print("\n🌌 检查天空点云是否足够大...")
    
    # 判断输入类型并获取DataFrame
    df_cloud = cloud.points if isinstance(cloud, PyntCloud) else cloud
    df_sky = sky_cloud.points if isinstance(sky_cloud, PyntCloud) else sky_cloud
    
    # 计算cloud的外接box
    cloud_x_range = df_cloud['x'].max() - df_cloud['x'].min()
    cloud_y_range = df_cloud['y'].max() - df_cloud['y'].min()
    cloud_z_max = df_cloud['z'].max()
    
    # 计算sky_cloud的外接box (只考虑Z>0的部分)
    sky_positive_z = df_sky[df_sky['z'] > 0]
    if len(sky_positive_z) == 0:
        print("⚠️ 警告: sky_cloud中没有Z>0的点，直接合并")
        return merge_ply_strategy_simple(cloud, sky_cloud)
    
    sky_x_range = sky_positive_z['x'].max() - sky_positive_z['x'].min()
    sky_y_range = sky_positive_z['y'].max() - sky_positive_z['y'].min()
    sky_z_max = sky_positive_z['z'].max()
    
    print(f"  Cloud外接box范围: X={cloud_x_range:.2f}m, Y={cloud_y_range:.2f}m, Z_max={cloud_z_max:.2f}m")
    print(f"  Sky外接box范围: X={sky_x_range:.2f}m, Y={sky_y_range:.2f}m, Z_max={sky_z_max:.2f}m")
    
    # 计算需要的扩大比例（取XYZ三个方向的最大比例）
    scale_x = cloud_x_range / sky_x_range if sky_x_range < cloud_x_range else 1.0
    scale_y = cloud_y_range / sky_y_range if sky_y_range < cloud_y_range else 1.0
    scale_z = cloud_z_max / sky_z_max if sky_z_max < cloud_z_max else 1.0
    
    # 取最大比例，并留出一些余量（1.2倍）
    scale_factor = max(scale_x, scale_y, scale_z)
    
    print(f"  XYZ比例: X={scale_x:.2f}, Y={scale_y:.2f}, Z={scale_z:.2f}")
    print(f"  最终扩大比例: {scale_factor:.2f}")
    
    # 复制sky_cloud的DataFrame
    df_sky_scaled = df_sky.copy()
    
    # 只对Z>0的点进行扩大
    if scale_factor > 1.0:
        print(f"  🔧 需要扩大天空点云...")
        mask_positive_z = df_sky_scaled['z'] > 0
        
        # 只扩大xyz坐标，其他属性保持不变
        df_sky_scaled.loc[mask_positive_z, 'x'] *= scale_factor
        df_sky_scaled.loc[mask_positive_z, 'y'] *= scale_factor
        df_sky_scaled.loc[mask_positive_z, 'z'] *= scale_factor
        
        scaled_count = mask_positive_z.sum()
        print(f"  ✅ 已扩大 {scaled_count:,} 个天空点（Z>0）")
    else:
        print(f"  ✅ 天空点云已足够大，无需扩大")

    return PyntCloud(df_sky_scaled)


def load_ply_gs(path):
    print("Reading PLY file from path:", path)
    plydata1 = PlyData.read(path)
    vertices = plydata1['vertex']
    
    # 打印所有可用的字段
    print(f"可用字段: {vertices.data.dtype.names}")
    print(f"总点数: {len(vertices):,}")
    
    positions = np.vstack([vertices['x'], vertices['y'], vertices['z']]).T
    print(f"Positions shape: {positions.shape}, 点数: {len(positions):,}")
    colors = np.vstack([vertices['red'], vertices['green'], vertices['blue']]).T / 255.0
    zero_matrix = np.zeros_like(positions)
    return BasicPointCloud(points=positions, colors=colors, normals=zero_matrix)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crop PLY point cloud by XY range")
    # parser.add_argument("--input_ply", type=str, required=True, help="Input PLY file path")
    # parser.add_argument("--output_ply", type=str, required=True, help="Output cropped PLY file path")
    parser.add_argument("--x_min", type=float, default=-70, help="Minimum X value")
    parser.add_argument("--x_max", type=float, default=70, help="Maximum X value")
    parser.add_argument("--y_min", type=float, default=-70, help="Minimum Y value")
    parser.add_argument("--y_max", type=float, default=70, help="Maximum Y value")
    parser.add_argument("--use_best_merge", action='store_true', help="使用择优融合策略（默认使用简单拼接）")
    parser.add_argument("--merge_threshold", type=float, default=0.02, help="择优融合的空间距离阈值")
    args = parser.parse_args()
    # gs_dir =  "/data/3dgs_data/"
    gs_dir =  "/data/3dgs_data_grid/"

    ply_sky = "/data/3dgs_data/model.ply"

    # load_ply_gs(ply_sky)

    sky_cloud = load_ply(ply_sky)

    # temp_file  = '/data/point_cloud.ply'
    # cloud = load_ply(temp_file)
    # cloud = crop_points_by_dis(cloud)
    # save_ply(cloud.points, "/data/point_cloud1.ply")

    # sky_cloud = sky_cloud_is_big_enough( cloud, sky_cloud )
    # merged_cloud_with_sky = merge_ply_strategy_simple( cloud, sky_cloud )

    # save_ply(merged_cloud_with_sky.points, "/data/3dgs_data/search_31.426000_120.619000_70.0_100.0m_resorted/3dgs_bg/point_cloud/iteration_599993/point_cloudnn.ply")

    # exit(0)

    data_list = get_all_session_dirs(gs_dir)
    # data_list =[
    #     '/data/3dgs_data/search_31.426000_120.619000_70.0_100.0m_resorted', 
    #     '/data/3dgs_data/search_31.426090_120.620308_70.0_100.0m_resorted',
    #     '/data/3dgs_data/search_31.425826_120.621359_50.0_100.0m_resorted', 
    #     '/data/3dgs_data/search_31.426329_120.619355_70.0_100.0m_resorted' 
    #     ]

    print("找到3DGS数据会话目录:")
    cloud_list = []
    cnt = 0
    for one_work_dir in data_list:
        if not ('_resorted' in one_work_dir):
            continue
        print(f"处理目录: {one_work_dir}")
        # continue
        # if cnt >= 9:
        #     break
        # cnt += 1

        # out_3dgs_ply = one_work_dir + "/3dgs_bg/point_cloud/iteration_9993/point_cloud.ply"
        out_3dgs_ply = one_work_dir + "/3dgs/point_cloud/iteration_599993/point_cloud.ply"
        if not os.path.exists(out_3dgs_ply):
            print(f"Warning: {out_3dgs_ply} does not exist, skipping.")
            continue
        # continue

        pose_file = get_first_yaml_from_vehicle_pose(one_work_dir)
        cloud = load_ply(out_3dgs_ply)

        cloud = crop_points_by_xy(cloud, args.x_min, args.x_max, args.y_min, args.y_max)

        pose = load_pose_and_transform(pose_file)

        world_cloud = gs_ply_to_world_temp(cloud, pose)
        
        cloud_list.append(world_cloud)

        # 保存world_cloud到文件
        session_name = os.path.basename(one_work_dir)
        world_output_path = f"/data/exported_roi_data/temp/map_{session_name}.ply"
        save_ply(world_cloud.points, world_output_path)

    print(f"DONE\n{len(cloud_list)} 个点云已转换到world坐标系，开始合并...")
    # exit(0)

    # 合并所有点云并保存

    # 保存没有天空的结果
    args.use_best_merge = True
    merged_cloud = merge_ply(cloud_list, threshold=args.merge_threshold, use_best_of_both=args.use_best_merge)
    if merged_cloud is not None:
        # output_path_sky = "/data/merged_3dgs_map_no_sky.ply"
        # save_ply(merged_cloud.points, output_path_sky)
        # 保存 有天空的结果
        sky_cloud = sky_cloud_is_big_enough( merged_cloud, sky_cloud )
        merged_cloud_with_sky = merge_ply_strategy_simple( merged_cloud, sky_cloud )
        output_path_sky = "/data/merged_3dgs_map_with_sky.ply"
        save_ply(merged_cloud_with_sky.points, output_path_sky)
    else:
        print("Error: merged_cloud is None, skipping save for no-sky version")
 
    print("DONE\n")

    exit(0)
