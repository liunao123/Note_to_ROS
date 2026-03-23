#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from glob import glob
import os
import sys
import argparse
import ast
import numpy as np
import yaml
import json
import pandas as pd
from pathlib import Path
from pyntcloud import PyntCloud
from scipy.spatial import cKDTree
import time


# 定义全局offset
offset_map = np.array([0.0 , 0.0, 0.0])
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

def VoxelSampl(cloud, coarse_size=5.0, fine_size=0.01, min_points_per_leaf=10):
    """
    对点云先用coarse_size(如5m)做八叉体素分块，再对每个cell用fine_size(如0.01m)做体素滤波，最后合并所有点。
    若某个coarse体素(叶子节点)内点数小于min_points_per_leaf，则直接删除该叶子节点。
    Args:
        cloud: PyntCloud对象
        coarse_size: 八叉树体素分辨率(米)
        fine_size: cell内体素滤波分辨率(米)
        min_points_per_leaf: 叶子节点最小点数阈值，低于该值会被删除
    Returns:
        PyntCloud对象，采样后的点云
    """
    df = cloud.points.copy()
    print(f"VoxelSampl: 滤波前点数: {len(df):,}")
    pts = df[['x', 'y', 'z']].to_numpy(dtype=np.float64)
    if pts.shape[0] == 0:
        return PyntCloud(df)

    # 优先使用PCL实现；若不可用再回退到Open3D/原实现
    try:
        import pcl
        print(f"[INFO] used PCL ")

        # 1) coarse体素键统计：按coarse_size构造“叶子节点”，删除稀疏叶子
        voxel_idx = np.floor(pts / float(coarse_size)).astype(np.int64)
        unique_keys, inverse = np.unique(voxel_idx, axis=0, return_inverse=True)
        counts = np.bincount(inverse)

        total_leaf_nodes = unique_keys.shape[0]
        keep_leaf_mask = counts >= int(min_points_per_leaf)

        kept_indices = np.where(keep_leaf_mask[inverse])[0]
        removed_points = int(pts.shape[0] - kept_indices.shape[0])
        removed_leaf_nodes = 0
        if total_leaf_nodes > 0:
            removed_leaf_nodes = int((~keep_leaf_mask).sum())

        if kept_indices.shape[0] == 0:
            sampled_df = df.iloc[0:0].copy()
        else:
            # 2) 先不做fine层VoxelGrid降采样，直接保留coarse过滤后的点
            sampled_df = df.iloc[kept_indices].copy()

        print(
            f"VoxelSampl(PCL): 叶子节点总数={total_leaf_nodes}, 删除稀疏叶子节点={removed_leaf_nodes}, "
            f"删除点数={removed_points:,}, 阈值={min_points_per_leaf}"
        )
        print(f"VoxelSampl(PCL): 滤波后点数: {len(sampled_df):,}")
        return PyntCloud(sampled_df)

    except Exception as e:
        print(f"[WARN] PCL实现失败，尝试Open3D: {e}")
        try:
            import open3d as o3d

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)

            min_bound = pts.min(axis=0) - 1e-6
            max_bound = pts.max(axis=0) + 1e-6

            _, _, coarse_trace = pcd.voxel_down_sample_and_trace(
                float(coarse_size),
                min_bound,
                max_bound,
                False
            )

            total_leaf_nodes = len(coarse_trace)
            kept_indices_chunks = []
            removed_leaf_nodes = 0
            removed_points = 0
            for id_vec in coarse_trace:
                ids = np.asarray(id_vec, dtype=np.int64)
                if ids.size < int(min_points_per_leaf):
                    removed_leaf_nodes += 1
                    removed_points += ids.size
                    continue
                kept_indices_chunks.append(ids)

            if len(kept_indices_chunks) == 0:
                sampled_df = df.iloc[0:0].copy()
            else:
                kept_indices = np.unique(np.concatenate(kept_indices_chunks))
                kept_pts = pts[kept_indices]

                pcd_kept = o3d.geometry.PointCloud()
                pcd_kept.points = o3d.utility.Vector3dVector(kept_pts)

                min_bound_kept = kept_pts.min(axis=0) - 1e-6
                max_bound_kept = kept_pts.max(axis=0) + 1e-6

                _, _, fine_trace = pcd_kept.voxel_down_sample_and_trace(
                    float(fine_size),
                    min_bound_kept,
                    max_bound_kept,
                    False
                )

                sampled_indices_local = []
                for id_vec in fine_trace:
                    ids = np.asarray(id_vec, dtype=np.int64)
                    if ids.size > 0:
                        sampled_indices_local.append(ids[0])

                if len(sampled_indices_local) == 0:
                    sampled_df = df.iloc[0:0].copy()
                else:
                    sampled_indices_local = np.asarray(sampled_indices_local, dtype=np.int64)
                    sampled_indices = kept_indices[sampled_indices_local]
                    sampled_df = df.iloc[sampled_indices].copy()

            print(
                f"VoxelSampl(Open3D): 叶子节点总数={total_leaf_nodes}, 删除稀疏叶子节点={removed_leaf_nodes}, "
                f"删除点数={removed_points:,}, 阈值={min_points_per_leaf}"
            )
            print(f"VoxelSampl(Open3D): 滤波后点数: {len(sampled_df):,}")
            return PyntCloud(sampled_df)

        except Exception as e2:
            print(f"[WARN] Open3D也失败，回退到pandas实现: {e2}")

            voxel_idx = np.floor(df[['x', 'y', 'z']].values / coarse_size).astype(int)
            df['coarse_voxel'] = [tuple(idx) for idx in voxel_idx]

            voxel_groups = list(df.groupby('coarse_voxel'))
            total_leaf_nodes = len(voxel_groups)
            removed_leaf_nodes = 0
            removed_points = 0

            sampled_points = []
            for _, group in voxel_groups:
                if len(group) < min_points_per_leaf:
                    removed_leaf_nodes += 1
                    removed_points += len(group)
                    continue

                group_pts = group[['x', 'y', 'z']].values
                fine_voxel_idx = np.floor((group_pts - np.min(group_pts, axis=0)) / fine_size).astype(int)
                seen = {}
                for i, idx in enumerate(map(tuple, fine_voxel_idx)):
                    if idx not in seen:
                        seen[idx] = group.iloc[i]
                sampled_points.extend(seen.values())

            sampled_df = pd.DataFrame(sampled_points)
            print(
                f"VoxelSampl(Fallback): 叶子节点总数={total_leaf_nodes}, 删除稀疏叶子节点={removed_leaf_nodes}, "
                f"删除点数={removed_points:,}, 阈值={min_points_per_leaf}"
            )
            print(f"VoxelSampl(Fallback): 滤波后点数: {len(sampled_df):,}")
            if 'coarse_voxel' in sampled_df.columns:
                sampled_df = sampled_df.drop(columns=['coarse_voxel'])
            return PyntCloud(sampled_df)


def read_3dgs_camera_poses(cam_json_path, select_cam='cam1'):
    """
    读取3DGS训练时的cameras.json文件，返回相机位姿信息
    Args:
        cam_json_path: cameras.json文件路径
    Returns:
        cameras: 列表，每个元素为dict，包含相机的位姿等信息
    """
    if not os.path.exists(cam_json_path):
        print(f"[read_3dgs_camera_poses] 文件不存在: {cam_json_path}")
        return []
    with open(cam_json_path, 'r') as f:
        data = json.load(f)
    # 兼容不同格式，常见格式有 'cameras' 或直接是列表
    if isinstance(data, dict) and 'cameras' in data:
        cameras = data['cameras']
    elif isinstance(data, list):
        cameras = data
    else:
        cameras = [data]

    print(f"[read_3dgs_camera_poses] 读取到 {len(cameras)} 个相机位姿")
    # 只返回img_name包含select_cam的相机
    filtered_cameras = [cam for cam in cameras if select_cam in cam.get('img_name', '')]
    print(f"[read_3dgs_camera_poses] 过滤后剩余 {len(filtered_cameras)} 个{select_cam}相机")
    return filtered_cameras


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
    
    translation = load_lidar_pose_from_file(pose_path)["translation"]
    offset_utm = load_lidar_pose_from_file(pose_path)["offset_utm"]
    # print("translation:\n", translation)
    # print("offset_utm:\n", offset_utm)
    
    global offset_map, offset_map_initialized
    # 第一次调用时，用读取到的值初始化offset_map
    if not offset_map_initialized:
        offset_map = np.array(offset_utm)
        offset_map_initialized = True
        print("==========================")
        print("首次初始化offset_map:\n", offset_map)
        print("==========================")
    
    # print("==========================")
    # print("减去全局offset:\n", offset_map)
    # print("==========================")
    # 减去全局offset
    offset_map_reshaped = offset_map.reshape(1, 3)
    pose = np.array(offset_utm) - offset_map_reshaped
    # print("转换后的pose:\n", pose)
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

    gaussian_core_cols = {
        'x', 'y', 'z',
        'f_dc_0', 'f_dc_1', 'f_dc_2',
        'opacity',
        'scale_0', 'scale_1', 'scale_2',
        'rot_0', 'rot_1', 'rot_2', 'rot_3'
    }

    def _is_3dgs_gaussian_schema(df):
        return gaussian_core_cols.issubset(set(df.columns))

    def _align_to_target_gaussian_schema(df, target_columns):
        aligned = df.copy()
        is_non_gaussian_like = not _is_3dgs_gaussian_schema(aligned)

        # 优先将RGB映射为3DGS的DC系数，避免纯色点云缺少f_dc字段
        if all(c in target_columns for c in ['f_dc_0', 'f_dc_1', 'f_dc_2']) and \
           not all(c in aligned.columns for c in ['f_dc_0', 'f_dc_1', 'f_dc_2']):
            if all(c in aligned.columns for c in ['red', 'green', 'blue']):
                rgb = aligned[['red', 'green', 'blue']].to_numpy(dtype=np.float32)
                if rgb.size > 0 and np.nanmax(rgb) > 1.0:
                    rgb = rgb / 255.0
                rgb = np.clip(rgb, 0.0, 1.0)
                c0 = 0.28209479177387814
                aligned['f_dc_0'] = (rgb[:, 0] - 0.5) / c0
                aligned['f_dc_1'] = (rgb[:, 1] - 0.5) / c0
                aligned['f_dc_2'] = (rgb[:, 2] - 0.5) / c0

        # 补齐缺失的高斯属性默认值
        for col in target_columns:
            if col in aligned.columns:
                continue
            if col == 'opacity':
                aligned[col] = 0.0
            elif col in ['scale_0', 'scale_1', 'scale_2']:
                aligned[col] = -1.0 if is_non_gaussian_like else -6.0
                # aligned[col] = 4.0 if is_non_gaussian_like else -6.0  # ok
            elif col == 'rot_0':
                aligned[col] = 1.0
            elif col in ['rot_1', 'rot_2', 'rot_3']:
                aligned[col] = 0.0
            else:
                aligned[col] = 0.0

        # 严格按目标schema输出，避免出现字段混杂
        return aligned[target_columns]

    df1_is_gaussian = _is_3dgs_gaussian_schema(df1)
    df2_is_gaussian = _is_3dgs_gaussian_schema(df2)

    if df1_is_gaussian or df2_is_gaussian:
        # 优先对齐到3DGS高斯字段（以第一个高斯输入为模板）
        template_df = df1 if df1_is_gaussian else df2
        target_columns = list(template_df.columns)
        print(f"   检测到3DGS高斯schema，按模板字段对齐: {len(target_columns)} 列")

        aligned_df1 = _align_to_target_gaussian_schema(df1, target_columns)
        aligned_df2 = _align_to_target_gaussian_schema(df2, target_columns)
        merged_df = pd.concat([aligned_df1, aligned_df2], ignore_index=True)
    else:
        # 无高斯schema时保持原逻辑
        merged_df = pd.concat([df1, df2], ignore_index=True)
    
    print(f"   融合后: {len(merged_df):,}")
    
    return PyntCloud(merged_df)

def compute_point_quality(df):
    """计算每个点的质量分数：更不透明且更小的高斯优先。"""
    if all(col in df.columns for col in ['opacity', 'scale_0', 'scale_1', 'scale_2']):
        opacity = 1 / (1 + np.exp(-df['opacity'].to_numpy()))
        scales = np.exp(df[['scale_0', 'scale_1', 'scale_2']].to_numpy())
        return opacity * (1.0 / (scales.mean(axis=1) + 1e-6))
    if 'opacity' in df.columns:
        return 1 / (1 + np.exp(-df['opacity'].to_numpy()))
    return np.ones(len(df), dtype=np.float64)


def merge_two_clouds_quality(df1, df2, threshold=0.05, margin=5.0):
    """在重叠区域按质量择优融合两个点云，非重叠区域全部保留。"""
    if len(df1) == 0:
        return df2.copy().reset_index(drop=True)
    if len(df2) == 0:
        return df1.copy().reset_index(drop=True)

    xyz1 = df1[['x', 'y', 'z']].to_numpy()
    xyz2 = df2[['x', 'y', 'z']].to_numpy()

    min1, max1 = xyz1.min(axis=0), xyz1.max(axis=0)
    min2, max2 = xyz2.min(axis=0), xyz2.max(axis=0)
    intersect_min = np.maximum(min1, min2) - margin
    intersect_max = np.minimum(max1, max2) + margin

    mask1 = np.all((xyz1 >= intersect_min) & (xyz1 <= intersect_max), axis=1)
    mask2 = np.all((xyz2 >= intersect_min) & (xyz2 <= intersect_max), axis=1)
    idx1_in = np.where(mask1)[0]
    idx2_in = np.where(mask2)[0]

    if len(idx1_in) == 0 or len(idx2_in) == 0:
        return pd.concat([df1, df2], ignore_index=True)

    xyz1_in = xyz1[idx1_in]
    xyz2_in = xyz2[idx2_in]
    quality1 = compute_point_quality(df1)
    quality2 = compute_point_quality(df2)

    tree1 = cKDTree(xyz1_in, leafsize=32)
    distances, nearest_idx_local = tree1.query(xyz2_in)
    duplicate_mask = distances < threshold

    keep1 = np.ones(len(df1), dtype=bool)
    keep2 = np.ones(len(df2), dtype=bool)
    replaced_count = 0

    for i_local, is_dup in enumerate(duplicate_mask):
        if not is_dup:
            continue
        i2 = idx2_in[i_local]
        i1 = idx1_in[nearest_idx_local[i_local]]
        if quality2[i2] > quality1[i1]:
            keep1[i1] = False
            replaced_count += 1
        else:
            keep2[i2] = False

    print(f"    重叠区域候选点: cloud1={len(idx1_in):,}, cloud2={len(idx2_in):,}, 近邻冲突={int(duplicate_mask.sum()):,}")
    print(f"    质量择优替换: 用cloud2替换cloud1 {replaced_count:,} 个点")

    return pd.concat([df1[keep1], df2[keep2]], ignore_index=True)


def filter_cloud_by_center_radius(df, center, grid_radius):
    """按点云中心的XY半径过滤点。"""
    points = df[['x', 'y', 'z']].to_numpy()
    dist_to_center = np.sqrt((points[:, 0] - center[0])**2 + (points[:, 1] - center[1])**2)
    keep_mask = dist_to_center <= grid_radius
    return df[keep_mask].copy(), int((~keep_mask).sum())


def nearest_center_owner_mask(points_xyz, centers, owner_idx):
    """返回每个点是否归属于owner_idx中心（按XY最近中心）。"""
    centers_array = np.asarray(centers)
    distances_to_all = np.sqrt(
        (points_xyz[:, 0:1] - centers_array[:, 0])**2 +
        (points_xyz[:, 1:2] - centers_array[:, 1])**2
    )
    nearest_center_idx = np.argmin(distances_to_all, axis=1)
    return nearest_center_idx == owner_idx


def sample_interval(cloud, interval=1):
    """按固定间隔对点云降采样。"""
    if cloud is None:
        return None

    interval = int(interval) if interval is not None else 1
    if interval <= 1:
        return cloud

    df = cloud.points if isinstance(cloud, PyntCloud) else cloud
    sampled_df = df.iloc[::interval].reset_index(drop=True)
    print(f"sample_interval: interval={interval}, 点数 {len(df):,} -> {len(sampled_df):,}")

    if isinstance(cloud, PyntCloud):
        return PyntCloud(sampled_df)
    return sampled_df


# 每个点 到其对应控制点（点云box的中心）的距离比到其他控制点更近
def merge_ply_voronoi(cloud_list, grid_radius=1000.0, overlap_threshold=0.05, overlap_margin=50.0, overlap_mode='single'):
    """
    智能合并多个点云：先按各自中心半径裁剪，再在重叠区域按质量择优融合。
    Args:
        cloud_list: PyntCloud对象列表
        grid_radius: 每个点云的有效网格半径（米），超出此半径的点会被去除
    Returns:
        合并后的PyntCloud对象
    """
    if not cloud_list:
        print("Warning: empty cloud list")
        return None

    if len(cloud_list) == 1:
        return cloud_list[0]

    if overlap_mode not in ['quality', 'single']:
        raise ValueError(f"overlap_mode must be 'quality' or 'single', got: {overlap_mode}")

    print(f"\n📊 合并 {len(cloud_list)} 个点云（mode={overlap_mode}, 网格半径={grid_radius}m, 重叠阈值={overlap_threshold}m）...")

    cloud_dfs = [cloud.points if isinstance(cloud, PyntCloud) else cloud for cloud in cloud_list]
    centers = [np.array([df['x'].mean(), df['y'].mean(), df['z'].mean()]) for df in cloud_dfs]

    filtered_clouds = []
    for i, (df, center) in enumerate(zip(cloud_dfs, centers)):
        print(f"\n  处理点云 {i+1}/{len(cloud_dfs)}:")
        print(f"    原始点数: {len(df):,}")
        print(f"    点云中心: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")

        filtered_df, removed_by_radius = filter_cloud_by_center_radius(df, center, grid_radius)
        if removed_by_radius > 0:
            print(f"    去除超出自己grid半径的点: {removed_by_radius:,}")

        if len(filtered_df) == 0:
            print("    警告：过滤后无点保留！")
        else:
            if overlap_mode == 'single':
                points_xyz = filtered_df[['x', 'y', 'z']].to_numpy()
                owner_mask = nearest_center_owner_mask(points_xyz, centers, i)
                removed_by_owner = int((~owner_mask).sum())
                if removed_by_owner > 0:
                    print(f"    去除更靠近其他点云中心的点: {removed_by_owner:,}")
                filtered_df = filtered_df[owner_mask].copy()

            print(f"    保留点数(仅半径过滤后): {len(filtered_df):,}")
            print(f"    去除总数: {len(df) - len(filtered_df):,}")

        debug_output_path = f"/data/exported_roi_data/temp/filtered_cloud_{i+1}.ply"
        os.makedirs(os.path.dirname(debug_output_path), exist_ok=True)
        save_ply(filtered_df, debug_output_path)
        print(f"    已保存过滤后的点云到: {debug_output_path}")

        filtered_clouds.append(filtered_df)

    if overlap_mode == 'quality':
        print(f"\n  开始质量择优融合所有过滤后的点云...")
        merged_df = filtered_clouds[0].reset_index(drop=True)
        for i in range(1, len(filtered_clouds)):
            print(f"  融合进度: {i+1}/{len(filtered_clouds)}")
            merged_df = merge_two_clouds_quality(
                merged_df,
                filtered_clouds[i].reset_index(drop=True),
                threshold=overlap_threshold,
                margin=overlap_margin
            )
            print(f"    当前融合后点数: {len(merged_df):,}")
    else:
        print("\n  使用single模式：按最近中心归属后直接拼接...")
        merged_df = pd.concat(filtered_clouds, ignore_index=True)

    print(f"  ✅ 合并完成，最终点数: {len(merged_df):,}")
    return PyntCloud(merged_df)


def merge_ply(cloud_list, threshold=0.025, use_best_of_both=False, interval=1):
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
        # ...existing code...
        df1 = cloud_list[0].points
        merged_data = {col: df1[col].values for col in df1.columns}
        class PropMock:
            def __init__(self, name):
                self.name = name
        props = [PropMock(col) for col in df1.columns]
        for i in range(1, len(cloud_list)):
            print(f"\n正在融合第 {i+1}/{len(cloud_list)} 个点云...")
            df2 = cloud_list[i].points
            data2 = {col: df2[col].values for col in df2.columns}
            merged_data = strategy_best_of_both(merged_data, data2, props, threshold=threshold)
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

    # interval采样逻辑统一放到最后
    # interval = 5 # 可调整采样间隔
    print(f"   interval参数: {interval}")
    if interval is not None and interval > 1:
        sampled_df = merged_cloud.points.iloc[::interval].reset_index(drop=True)
        merged_cloud = PyntCloud(sampled_df)
        print(f"采样后点数: {len(sampled_df):,}")

    return merged_cloud


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

    # 1. 获取cloud的平面中心点
    center_x = (df_cloud['x'].max() + df_cloud['x'].min()) / 2
    center_y = (df_cloud['y'].max() + df_cloud['y'].min()) / 2

    # 将 cloud的Z轴最低点 后面与 sky_cloud的Z轴最低点对齐， 这样sky就尽可能远离cloud
    center_z = df_cloud['z'].min()
    center = np.array([center_x, center_y, center_z])
    print(f"  Cloud中心点: {center}")

    # 2. 将sky_cloud的原点平移到cloud的中心点
    df_sky_shifted = df_sky.copy()
    # 计算 sky_cloud 原点（用全部点的中心）
    # sky_center_x = (df_sky['x'].max() + df_sky['x'].min()) / 2
    # sky_center_y = (df_sky['y'].max() + df_sky['y'].min()) / 2
    # sky_center_z = (df_sky['z'].max() + df_sky['z'].min()) / 2
    # sky_center = np.array([sky_center_x, sky_center_y, sky_center_z])
    # 本来原点就在(0,0,0) 不用计算
    sky_center = np.array([0, 0, 0])
    print(f"  SkyCloud原点: {sky_center}")

    shift = center - sky_center
    print(f"  平移向量: {shift}")
    df_sky_shifted[['x', 'y', 'z']] = df_sky_shifted[['x', 'y', 'z']] + shift

    # 3. 判断sky_cloud是否完全包含cloud（只考虑Z>0的sky_cloud）
    shifted_sky_positive_z = df_sky_shifted[df_sky_shifted['z'] > -1000]
    sky_x_min, sky_x_max = shifted_sky_positive_z['x'].min(), shifted_sky_positive_z['x'].max()
    sky_y_min, sky_y_max = shifted_sky_positive_z['y'].min(), shifted_sky_positive_z['y'].max()
    sky_z_min, sky_z_max = shifted_sky_positive_z['z'].min(), shifted_sky_positive_z['z'].max()

    cloud_x_min, cloud_x_max = df_cloud['x'].min(), df_cloud['x'].max()
    cloud_y_min, cloud_y_max = df_cloud['y'].min(), df_cloud['y'].max()
    cloud_z_min, cloud_z_max = df_cloud['z'].min(), df_cloud['z'].max()

    print(f"  Cloud外接box: X=[{cloud_x_min:.2f},{cloud_x_max:.2f}], Y=[{cloud_y_min:.2f},{cloud_y_max:.2f}], Z=[{cloud_z_min:.2f},{cloud_z_max:.2f}]")
    print(f"  Sky外接box: X=[{sky_x_min:.2f},{sky_x_max:.2f}], Y=[{sky_y_min:.2f},{sky_y_max:.2f}], Z=[{sky_z_min:.2f},{sky_z_max:.2f}]")

    # 4. 如果sky_cloud不包含cloud，则扩大sky_cloud（只对Z>0的点）
    scale_x = (cloud_x_max - cloud_x_min) / (sky_x_max - sky_x_min) if (sky_x_max - sky_x_min) < (cloud_x_max - cloud_x_min) else 1.0
    scale_y = (cloud_y_max - cloud_y_min) / (sky_y_max - sky_y_min) if (sky_y_max - sky_y_min) < (cloud_y_max - cloud_y_min) else 1.0
    scale_z = (cloud_z_max - cloud_z_min) / (sky_z_max - sky_z_min) if (sky_z_max - sky_z_min) < (cloud_z_max - cloud_z_min) else 1.0
    scale_factor = max(scale_x, scale_y, scale_z)
    print(f"  XYZ比例: X={scale_x:.2f}, Y={scale_y:.2f}, Z={scale_z:.2f}")
    print(f"  最终扩大比例: {scale_factor:.2f}")

    if scale_factor > 1.0:
        print(f"  🔧 需要扩大天空点云...")
        mask_positive_z = df_sky_shifted['z'] > 0
        # 以cloud中心为缩放中心
        df_sky_shifted.loc[mask_positive_z, ['x', 'y', 'z']] = (
            (df_sky_shifted.loc[mask_positive_z, ['x', 'y', 'z']] - center) * scale_factor + center
        )
        scaled_count = mask_positive_z.sum()
        print(f"  ✅ 已扩大 {scaled_count:,} 个天空点 Z>0 ")
    else:
        print(f"  ✅ 天空点云已足够大，无需扩大")

    return PyntCloud(df_sky_shifted)


def parse_namespace_cfg(cfg_content):
    """将 Namespace(a=1, b='x') 解析为 dict。"""
    expr = ast.parse(cfg_content.strip(), mode='eval')
    if not isinstance(expr.body, ast.Call) or not isinstance(expr.body.func, ast.Name) or expr.body.func.id != 'Namespace':
        raise ValueError("cfg_args 格式不是 Namespace(...)")

    parsed = {}
    for kw in expr.body.keywords:
        parsed[kw.arg] = ast.literal_eval(kw.value)
    return parsed


def format_namespace_cfg(cfg_dict):
    """将 dict 重新格式化为 Namespace(...) 字符串。"""
    items = []
    for key, value in cfg_dict.items():
        items.append(f"{key}={repr(value)}")
    return f"Namespace({', '.join(items)})"


def save_cfg_args(output_root, template_cfg_path):
    """
    在output_root文件夹下生成cfg_args文件。
    除路径相关字段外，其余配置从待合并项目的3dgs/cfg_args中读取。
    Args:
        output_root: 输出根目录
        template_cfg_path: 模板cfg_args路径
    """
    abs_output_root = os.path.abspath(output_root)
    os.makedirs(abs_output_root, exist_ok=True)

    with open(template_cfg_path, 'r', encoding='utf-8') as f:
        template_content = f.read().strip()

    cfg_dict = parse_namespace_cfg(template_content)

    # 路径相关字段保持当前合并输出逻辑不变
    cfg_dict['source_path'] = abs_output_root
    cfg_dict['model_path'] = abs_output_root
    if 'source_path_slave' in cfg_dict:
        cfg_dict['source_path_slave'] = None

    cfg_content = format_namespace_cfg(cfg_dict)

    cfg_path = os.path.join(output_root, "cfg_args")
    with open(cfg_path, 'w', encoding='utf-8') as f:
        f.write(cfg_content)

    print(f"模板cfg_args来源: {template_cfg_path}")
    print(f"已保存cfg_args到 {cfg_path}")
    print("cfg_args内容如下:")
    print(cfg_content)


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

# 提取时间戳，按时间戳排序后重新分配id
def extract_timestamp(img_name):
    # 例：0_1766199762.900_cam5.jpg
    parts = img_name.split('_')
    if len(parts) >= 3:
        return float(parts[1])
    return 0.0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crop PLY point cloud by XY range")
    # parser.add_argument("--input_ply", type=str, required=True, help="Input PLY file path")
    # parser.add_argument("--output_ply", type=str, required=True, help="Output cropped PLY file path")
    parser.add_argument("--x_min", type=float, default=-150, help="Minimum X value")
    parser.add_argument("--x_max", type=float, default=150, help="Maximum X value")
    parser.add_argument("--y_min", type=float, default=-150, help="Minimum Y value")
    parser.add_argument("--y_max", type=float, default=150, help="Maximum Y value")
    parser.add_argument("--use_best_merge", action='store_true', help="使用择优融合策略（默认使用简单拼接）")
    parser.add_argument("--merge_threshold", type=float, default=0.05, help="择优融合的空间距离阈值")
    parser.add_argument("--interval", type=int, default=1, help="按间隔降采样merged_cloud , 1表示不降采样")
    parser.add_argument("--overlap_mode", type=str, default="single", choices=["quality", "single"], help="重叠区处理模式：quality择优融合，single按最近中心仅保留一个点云")
    args = parser.parse_args()
    gs_dir =  "/data/3dgs_test_sample/"
    # gs_dir =  "/data/3dgs_data_grid/"

    # merge 结果的保存 文件夹
    output_root = "/data/3dgs_merged_result/"
    Path(output_root).mkdir(parents=True, exist_ok=True)

    ply_sky = "/data/3dgs_data/sky_model.ply"

    sky_cloud = load_ply(ply_sky)

    # 单独merge两个高斯模型
    # gs_ply_path_lidar = "/data/3dgs_data_grid/search_31.425798_120.626449_50.0_75.0m_resorted/sparse/post/gs_init.ply"
    # gs_ply_path_vision = "/data/3dgs_data_grid/search_31.425798_120.626449_50.0_75.0m_resorted/sparse/refined_points.ply"
    # gs_ply_path_new = "/data/3dgs_data_grid/search_31.425798_120.626449_50.0_75.0m_resorted/sparse/post/gs_init_new.ply"

    # cloud1 = load_ply(gs_ply_path_lidar)
    # print(f"1, 点数: {len(cloud1.points):,}")
    # cloud2 = load_ply(gs_ply_path_vision)
    # print(f"2, 点数: {len(cloud2.points):,}")

    # merged_cloud = merge_ply_strategy_simple( cloud1, cloud2 )
    # save_ply(merged_cloud.points, gs_ply_path_new)
    # print(f"已保存融合后的点云到: {gs_ply_path_new}, 点数: {len(merged_cloud.points):,}")
    # exit(0)


    # 自动获取所有分块session目录
    # data_list = get_all_session_dirs(gs_dir)

    # 1-0  1-1 1-1  
    data_list =[
        '/data/3dgs_data_grid/search_31.424902_120.626466_50.0_70.0m_resorted/', # 第一个起点，后续也以这个为第一个进行合并
        '/data/3dgs_data_grid/search_31.425801_120.626444_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.426709_120.626421_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.427538_120.626400_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.424299_120.627533_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.427396_120.627455_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.424037_120.628592_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.427029_120.628516_50.0_70.0m_resorted/',

        '/data/3dgs_data_grid/search_31.423825_120.629649_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.424728_120.629626_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.425631_120.629603_50.0_70.0m_resorted/',
        '/data/3dgs_data_grid/search_31.426474_120.629582_50.0_70.0m_resorted/'

        ]
    # exit(0)
    # 没有将gsinit.ply裁剪到方框，全部点都保留的 merge测试
    # 7-0 - 7-3 的 4 个session，位置比较接近
    # data_list =[
    #     # '/data/3dgs_data_grid/search_31.424897_120.626472_50.0_75.0m_resorted/', # 1 sh_degree
    #     # '/data/3dgs_data_grid/search_31.425798_120.626449_50.0_75.0m_resorted/',  # 0
    #     # '/data/3dgs_data_grid/search_31.426704_120.626426_50.0_75.0m_resorted/',  # 0
    #     # '/data/3dgs_data_grid/search_31.427533_120.626405_50.0_75.0m_resorted/',
    #     # '/data/3dgs_data_grid/search_31.424854_120.625416_50.0_70.0m_resorted/',
    #     # '/data/3dgs_data_grid/search_31.425120_120.624357_50.0_70.0m_resorted/',
    #     ]
    
    # 保存cfg_args文件
    # sh_degree 要和原来的保持一致  # TODO 用最小的那个 行不行？ 
    template_cfg_path = os.path.join(data_list[0], "3dgs/cfg_args")
    if not os.path.exists(template_cfg_path):
        raise FileNotFoundError(f"模板cfg_args不存在: {template_cfg_path}")
        exit(0)
    save_cfg_args(output_root, template_cfg_path)

    print("找到3DGS数据会话目录:")
    cloud_list = []
    all_cameras = []
    session_names = []
    for idx, one_work_dir in enumerate(data_list):
        # if not ('_resorted' in one_work_dir):
        #     continue
        # if idx > 2:
        #     break
        print(f"处理目录: {one_work_dir}: idx={idx+1}/{len(data_list)}")
        # continue
        session_names.append(os.path.basename(one_work_dir))
        out_3dgs_ply = os.path.join(one_work_dir, "3dgs/point_cloud/iteration_599993/point_cloud.ply")
        # out_3dgs_ply = os.path.join(one_work_dir, "sparse/post/gs_init.ply")
        cam_3dgs_json = os.path.join(one_work_dir, "3dgs/cameras.json")
        if not os.path.exists(out_3dgs_ply):
            print(f"Warning: {out_3dgs_ply} does not exist, skipping.")
            # exit(0)
            continue
        pose_file = get_first_yaml_from_vehicle_pose(one_work_dir)
        pose = load_pose_and_transform(pose_file)
        # 点云转换到world坐标系
        cloud = load_ply(out_3dgs_ply)
        print(f"原始gs数 cloud {cloud.points.shape[0]} 个点")

        # ! 这里原始的cloud可能有 sky box ,后续merge之前需要先去除 sky box，
        # ! 最后merge 之后， 再单独加一个足够大的sky
        # cloud = crop_points_by_xy(cloud, args.x_min, args.x_max, args.y_min, args.y_max)

        world_cloud = gs_ply_to_world_temp(cloud, pose)
        cloud_list.append(world_cloud)
        
        # 保存world_cloud到文件
        # session_name = os.path.basename(one_work_dir)
        # world_output_path = f"/data/exported_roi_data/temp/map_{idx}.ply"
        # save_ply(world_cloud.points, world_output_path)
        # print(f"871 已保存转换到world坐标系的点云到 {world_cloud.points.shape[0]} 个点")
        # print(f"已保存world_cloud到 {world_output_path}")
        # continue

        # 相机轨迹转换
        # 保留哪一个相机的位姿，cam1、cam5等，
        # 默认cam5 是往前看的，cam1是往后看的，其他cam2/3/4是左右的，可以根据需要选择
        cameras = read_3dgs_camera_poses(cam_3dgs_json, select_cam='cam5')
        for idy, cam in enumerate(cameras):
            # if idy == 0:
            #     print(f"原始第一个相机位姿:\n{cam}")
            # if idy % 1:
            #     continue

            if 'position' in cam:
                pos = cam['position']
                pos_aligned = np.array(pos)
                x = pos_aligned[0] + pose[0][0]
                y = pos_aligned[1] + pose[0][1]
                z = pos_aligned[2] + pose[0][2]
                aligned = [x, y, z]
                cam['position'] = aligned
            all_cameras.append(cam)
    # 合并所有点云
    print(f"DONE\n{len(all_cameras)} 个相机 ...")
    print(f"DONE\n{len(cloud_list)} 个点云已转换到world坐标系，开始合并...")
    # exit(0)

    merged_cloud = merge_ply_voronoi(cloud_list, grid_radius=500.0, overlap_mode=args.overlap_mode)
    
    # args.interval = 2
    # merged_cloud = sample_interval(merged_cloud, interval=args.interval)

    # args.use_best_merge = True
    # # args.use_best_merge = False
    # merged_cloud = merge_ply(cloud_list, threshold=args.merge_threshold, use_best_of_both=args.use_best_merge, interval=args.interval)

    # 体素采样
    # t_voxel_start = time.time()
    # merged_cloud = VoxelSampl(merged_cloud)
    # t_voxel_end = time.time()
    # print(f"⏱️  体素采样耗时: {t_voxel_end - t_voxel_start:.2f}秒")

    os.makedirs(output_root, exist_ok=True)
    output_ply_path = os.path.join(output_root, "point_cloud/iteration_699993/point_cloud.ply")
    # output_ply_path = os.path.join(output_root, "point_cloud/point_cloud_all.ply")
    os.makedirs(os.path.dirname(output_ply_path), exist_ok=True)

    # if merged_cloud is not None:
    if False:
        # 保存 有天空的结果
        print("start merge eith sky cloud ...")
        sky_cloud = sky_cloud_is_big_enough( merged_cloud, sky_cloud )
        merged_cloud_with_sky = merge_ply_strategy_simple( merged_cloud, sky_cloud )
        save_ply(merged_cloud_with_sky.points, output_ply_path)
    else:
        save_ply(merged_cloud.points, output_ply_path)
        print("Error: merged_cloud is None, skipping save for no-sky version")
    # exit(0)

    # 先提取时间戳
    for cam in all_cameras:
        cam['timestamp'] = extract_timestamp(cam.get('img_name', ''))
    # 按时间戳排序
    all_cameras.sort(key=lambda x: x['timestamp'])
    # 重新分配id
    for idx, cam in enumerate(all_cameras):
        cam['id'] = idx
        del cam['timestamp']
    merged_json_path = os.path.join(output_root, "cameras.json")

    with open(merged_json_path, "w", encoding="utf-8") as f:
        json.dump(all_cameras, f, ensure_ascii=False, indent=2)
    
    print(f"已保存所有{len(all_cameras)}个 cameras pose 到 {merged_json_path}")

    # 可视化所有相机位置
    import matplotlib.pyplot as plt
    positions = [cam['position'] for cam in all_cameras if 'position' in cam]
    positions = np.array(positions)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(1, 1, 1)
    ax.scatter(positions[:,0], positions[:,1], c='b', marker='o', s=1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Camera Positions Top-Down View')
    ax.axis('equal')
    ax.grid(True)
    out_img = os.path.join(output_root, "camera_positions_topdown.png")
    fig.tight_layout()
    fig.savefig(out_img)
    plt.close(fig)
    print(f"已保存相机可视化图片到 {out_img}")
    # 拷贝其它可视化所需文件（如config、模型等）
    # 可根据实际需求补充
    print("DONE\n")
