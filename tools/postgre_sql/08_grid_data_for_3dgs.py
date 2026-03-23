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
import pcl
import cv2

# 使用qt.load_config的统一配置加载
from qt.load_config import load_config
from qt.box_projection import (
    get_all_session_dirs,
    load_lidar_pose_from_file,
    utm_to_lonlat
)

def save_pose_to_pcd(xyz_array, save_path):
    """
    将XYZ坐标数组保存为PCD文件
    :param xyz_array: numpy数组/列表，形状为 (N, 3)，每行是[x, y, z]
    :param save_path: 保存路径，如 "poses_cloud.pcd"
    """
    # 1. 统一转换为float32类型的numpy数组（PCL要求）
    if not isinstance(xyz_array, np.ndarray):
        xyz_array = np.array(xyz_array, dtype=np.float32)
    else:
        xyz_array = xyz_array.astype(np.float32)
    
    # 2. 校验维度（必须是N行3列）
    if len(xyz_array) == 0:
        print("[WARN] 没有有效的XYZ坐标，跳过保存PCD")
        return
    if xyz_array.shape[1] != 3:
        raise ValueError("输入必须是N行3列的XYZ坐标数组！")
    
    # 3. 创建PCL点云对象并导入数据
    cloud = pcl.PointCloud()
    cloud.from_array(xyz_array)
    
    # 4. 保存为PCD文件
    pcl.save(cloud, save_path)

    print(f"✅ 位姿点云已成功保存到: {save_path}")
    print(f"📊 点云数量: {cloud.size} 个点")
    return cloud

class PointCloudGridDivider:
    class PointWithIndex:
        def __init__(self, point, index):
            self.point = point
            self.index = index

    class GridBoundary:
        def __init__(self, min_x, max_x, min_y, max_y, grid_i, grid_j, point_count):
            self.min_x = min_x
            self.max_x = max_x
            self.min_y = min_y
            self.max_y = max_y
            self.grid_i = grid_i
            self.grid_j = grid_j
            self.point_count = point_count

    def __init__(self):
        self.base_grid_size = 100.0
        self.max_aspect_ratio = 1.25
        self.min_points_per_grid = 20
        self.use_voxel_filter = False
        self.voxel_leaf_size = 0.5
        self.max_test_points = 0
        self.output_dir = "./pcd_grid/"
        self.utm_x0 = 0
        self.utm_y0 = 0
        self.utm_z0 = 0
        self.utm_zone = 51  # 默认UTM分区
        self.grids = []
        self.all_grids = []
        self.grid_boundaries = []
        self.total_grids = 0
        self.non_empty_grids = 0
        self.total_saved_points = 0
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.x_range = 0
        self.y_range = 0
    def setUTMZone(self, utm_zone):
        """设置UTM分区号"""
        self.utm_zone = utm_zone

    def setUTMOrigin(self, x, y, z):
        self.utm_x0 = x
        self.utm_y0 = y
        self.utm_z0 = z

    def setOutputDir(self, dir):
        self.output_dir = dir

    # ! 导出所有grid的中心点经纬度和大小到KML文件，矩形Polygon表示grid范围
    def export_batch_sh_for_3dgs(self, sh_path):
        """
        根据all_grids生成批量shell命令 写入sh_path文件。
        每个grid生成三行命令 lat/lon取中心点 roi_min/roi_max取x_size和y_size的较小/较大值。
        lat/lon保留6位小数，roi_min/roi_max保留1位小数。
        name按模板生成: search_{lat:.6f}_{lon:.6f}_{roi_distance_min}_{roi_distance_max}m
        """
        session_name_template = "search_{lat:.6f}_{lon:.6f}_{roi_distance_min}_{roi_distance_max}m"
        with open(sh_path, 'w', encoding='utf-8') as f:
            f.write('#!/bin/bash \n')
            f.write('########################################################## \n')
            f.write(f"# move this file to geo-database-engine path . then run ...... \n")
            f.write('########################################################## \n\n\n')
            for grid in self.all_grids:
                i, j, grid_size, lat, lon, x_size, y_size = grid
                roi_min = max(x_size, y_size) / 2.0
                roi_max = roi_min * 1.5
                lat_str = f"{lat:.6f}"
                lon_str = f"{lon:.6f}"
                roi_min_str = f"{roi_min:.1f}"
                roi_max_str = f"{roi_max:.1f}"
                f.write(f"#data for grid [{i},{j}] with {grid_size} points, center at ({lat_str}, {lon_str}), x_size={x_size:.1f}m, y_size={y_size:.1f}m\n")
                f.write(f"#roi_distance_min = max(x_size, y_size)/2.0\n")
                f.write(f"#roi_distance_max = roi_distance_min * 1.5\n")
                name = session_name_template.format(
                lat=lat,                      lon=lon,
                roi_distance_min=roi_min_str, roi_distance_max=roi_max_str 
                )
                f.write(f"#data name for 3dgs {name}\n")
                f.write(f"/usr/bin/python3 ./03_get_roi_data.py                            --lat {lat_str} --lon {lon_str} --roi_distance_min {roi_min_str} --roi_distance_max {roi_max_str}\n")
                f.write(f"/usr/bin/python3 ./04_resorted_by_time.py                        --lat {lat_str} --lon {lon_str} --roi_distance_min {roi_min_str} --roi_distance_max {roi_max_str}\n")
                f.write(f"/usr/bin/python3 ./05_get_roi_img_and_06_nu_sampling_for_3dgs.py --lat {lat_str} --lon {lon_str} --roi_distance_min {roi_min_str} --roi_distance_max {roi_max_str}\n")
                f.write("\n\n")
        print(f"已导出批量shell命令，执行可以生成每个块 3dgs 需要的数据结构: {sh_path}")

    def export_grids_to_kml(self, kml_path):
        """
        依次读取self.all_grids中的[i, j, grid_size, lat, lon, x_size, y_size]，以lat/lon为中心，x/y为边长，输出矩形Polygon到KML。
        并在中心点添加i,j,grid_size的label。
        """
        import pyproj
        geod = pyproj.Geod(ellps='WGS84')
        with open(kml_path, 'w', encoding='utf-8') as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
            f.write('  <Document>\n')
            for idx, grid in enumerate(self.all_grids):
                i, j, grid_size, lat, lon, x_size, y_size = grid
                # 以中心点lat/lon为中心，x_size/2, y_size/2为半边长，计算四角点
                n_lon, n_lat, _ = geod.fwd(lon, lat, 0, y_size/2)
                s_lon, s_lat, _ = geod.fwd(lon, lat, 180, y_size/2)
                nw_lon, nw_lat, _ = geod.fwd(n_lon, n_lat, 270, x_size/2)
                ne_lon, ne_lat, _ = geod.fwd(n_lon, n_lat, 90, x_size/2)
                sw_lon, sw_lat, _ = geod.fwd(s_lon, s_lat, 270, x_size/2)
                se_lon, se_lat, _ = geod.fwd(s_lon, s_lat, 90, x_size/2)
                coords = [
                    (nw_lon, nw_lat),
                    (ne_lon, ne_lat),
                    (se_lon, se_lat),
                    (sw_lon, sw_lat),
                    (nw_lon, nw_lat)
                ]
                kml_coords = "\n".join([f"{lon},{lat},0" for lon, lat in coords])
                # Polygon
                f.write('    <Placemark>\n')
                f.write(f'      <name>Grid_{i}_{j}</name>\n')
                f.write('      <Style><LineStyle><color>ff0000ff</color><width>2</width></LineStyle><PolyStyle><color>330000ff</color></PolyStyle></Style>\n')
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
                # Center label
                f.write('    <Placemark>\n')
                f.write(f'      <name>Grid[ {i},{j} ] {grid_size} pts</name>\n')
                f.write('      <Style><IconStyle><color>ff0000ff</color><scale>0.8</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/wht-blank.png</href></Icon></IconStyle></Style>\n')
                f.write('      <Point>\n')
                f.write(f'        <coordinates>{lon},{lat},0</coordinates>\n')
                f.write('      </Point>\n')
                f.write('    </Placemark>\n')
            f.write('  </Document>\n')
            f.write('</kml>\n')
        print(f"已导出所有grid到KML: {kml_path}")

    def create_grid_folders(self):
        """
        根据self.all_grids创建目录：
        grid_i_j_lat_lon_x_size_y_size
        """
        os.makedirs(self.output_dir, exist_ok=True)
        created_count = 0
        for grid in self.all_grids:
            i, j, grid_size, lat, lon, x_size, y_size = grid
            folder_name = f"grid_{i}_{j}_{lat:.6f}_{lon:.6f}_{x_size:.1f}_{y_size:.1f}"
            folder_path = os.path.join(self.output_dir, folder_name)
            os.makedirs(folder_path, exist_ok=True)
            created_count += 1
        print(f"已创建grid目录: {created_count} 个, 输出路径: {self.output_dir}")


    def divideAndSave(self, cloud):
        os.makedirs(self.output_dir, exist_ok=True)
        # 测试模式
        used_cloud = cloud
        if self.max_test_points > 0 and used_cloud.size > self.max_test_points:
            indices = np.arange(self.max_test_points)
            arr = used_cloud.to_array()
            used_cloud = pcl.PointCloud.PointXYZ()
            used_cloud.from_array(arr[indices])
        # 计算边界
        arr = used_cloud.to_array()
        min_pt = np.min(arr, axis=0)
        max_pt = np.max(arr, axis=0)
        self.min_x, self.min_y = min_pt[0], min_pt[1]
        self.max_x, self.max_y = max_pt[0], max_pt[1]
        self.x_range = self.max_x - self.min_x
        self.y_range = self.max_y - self.min_y
        # 按X排序
        sorted_points = [self.PointWithIndex(pt, idx) for idx, pt in enumerate(arr)]
        sorted_points.sort(key=lambda pwi: pwi.point[0])
        # X方向划分
        x_boundaries = [self.min_x]
        current_x = self.min_x
        max_x_grid_size = self.base_grid_size * self.max_aspect_ratio
        while current_x < self.max_x:
            remaining_x = self.max_x - current_x
            if remaining_x <= max_x_grid_size:
                break
            current_x += self.base_grid_size
            x_boundaries.append(current_x)
        x_boundaries.append(self.max_x)
        m = len(x_boundaries) - 1
        self.grids.clear()
        self.grid_boundaries.clear()
        self.total_grids = 0
        for i in range(m):
            x_strip_points = [pwi for pwi in sorted_points if x_boundaries[i] <= pwi.point[0] < x_boundaries[i+1]]
            if not x_strip_points:
                self.grids.append([])
                continue
            x_strip_points.sort(key=lambda pwi: pwi.point[1])
            strip_x_size = x_boundaries[i+1] - x_boundaries[i]
            max_y_grid_size = strip_x_size * self.max_aspect_ratio
            actual_y_grid_size = min(self.base_grid_size, max_y_grid_size)
            strip_grids = []
            strip_boundaries = []
            point_idx = 0
            j = 0
            while point_idx < len(x_strip_points):
                grid_start_y = x_strip_points[point_idx].point[1]
                grid_end_y = grid_start_y + actual_y_grid_size
                grid_cloud = []
                start_idx = point_idx
                while point_idx < len(x_strip_points) and x_strip_points[point_idx].point[1] < grid_end_y:
                    grid_cloud.append(x_strip_points[point_idx].point)
                    point_idx += 1
                if grid_cloud:
                    actual_min_y = x_strip_points[start_idx].point[1]
                    actual_max_y = x_strip_points[point_idx-1].point[1] if point_idx > 0 else actual_min_y
                    gb = self.GridBoundary(x_boundaries[i], x_boundaries[i+1], actual_min_y, actual_max_y, i, j, len(grid_cloud))
                    strip_grids.append(grid_cloud)
                    strip_boundaries.append(gb)
                    j += 1
            # 只在Y方向上执行一次合并/均分逻辑
            for k in range(len(strip_grids)-1):
                # 跳过空网格
                if len(strip_grids[k]) == 0 or len(strip_grids[k+1]) == 0:
                    continue
                # 只考虑空间上y区间相邻的grid
                y_overlap = min(strip_boundaries[k].max_y, strip_boundaries[k+1].max_y) - max(strip_boundaries[k].min_y, strip_boundaries[k+1].min_y)
                y_gap = max(strip_boundaries[k].min_y, strip_boundaries[k+1].min_y) - min(strip_boundaries[k].max_y, strip_boundaries[k+1].max_y)
                # 相邻条件：y_overlap>0 或 y_gap<self.base_grid_size*0.5
                if y_overlap > 0 or y_gap < self.base_grid_size * 0.5:
                    x_size1 = strip_boundaries[k].max_x - strip_boundaries[k].min_x
                    y_size1 = strip_boundaries[k].max_y - strip_boundaries[k].min_y
                    x_size2 = strip_boundaries[k+1].max_x - strip_boundaries[k+1].min_x
                    y_size2 = strip_boundaries[k+1].max_y - strip_boundaries[k+1].min_y
                    ratio1 = x_size1 / (y_size1 + 1e-6)
                    ratio2 = x_size2 / (y_size2 + 1e-6)
                    if ratio1 > self.max_aspect_ratio or ratio2 > self.max_aspect_ratio:
                        merged_points = strip_grids[k] + strip_grids[k+1]
                        merged_points.sort(key=lambda pt: pt[1])
                        n = len(merged_points)
                        half = n // 2
                        grid1 = merged_points[:half]
                        grid2 = merged_points[half:]
                        min_y1 = min(pt[1] for pt in grid1) if grid1 else strip_boundaries[k].min_y
                        max_y1 = max(pt[1] for pt in grid1) if grid1 else strip_boundaries[k].min_y
                        min_y2 = min(pt[1] for pt in grid2) if grid2 else strip_boundaries[k+1].max_y
                        max_y2 = max(pt[1] for pt in grid2) if grid2 else strip_boundaries[k+1].max_y
                        strip_grids[k] = grid1
                        strip_grids[k+1] = grid2
                        strip_boundaries[k].min_y = min_y1
                        strip_boundaries[k].max_y = max_y1
                        strip_boundaries[k].point_count = len(grid1)
                        strip_boundaries[k+1].min_y = min_y2
                        strip_boundaries[k+1].max_y = max_y2
                        strip_boundaries[k+1].point_count = len(grid2)
            self.grids.append(strip_grids)
            self.grid_boundaries.extend(strip_boundaries)
            self.total_grids += len(strip_grids)
        # X方向合并
        has_merge = True
        while has_merge:
            has_merge = False
            for i in range(len(self.grids)):
                for j in range(len(self.grids[i])):
                    if len(self.grids[i][j]) == 0 or len(self.grids[i][j]) >= self.min_points_per_grid:
                        continue
                    current_gb = None
                    for gb in self.grid_boundaries:
                        if gb.grid_i == i and gb.grid_j == j and gb.point_count > 0:
                            current_gb = gb
                            break
                    if not current_gb:
                        continue
                    merge_target_i = -1
                    merge_target_j = 0
                    merge_target_gb = None
                    if i > 0:
                        for k in range(len(self.grids[i-1])):
                            if len(self.grids[i-1][k]) == 0:
                                continue
                            for gb in self.grid_boundaries:
                                if gb.grid_i == i-1 and gb.grid_j == k and gb.point_count > 0:
                                    y_overlap = min(current_gb.max_y, gb.max_y) - max(current_gb.min_y, gb.min_y)
                                    y_gap = max(current_gb.min_y, gb.min_y) - min(current_gb.max_y, gb.max_y)
                                    if y_overlap > 0 or y_gap < self.base_grid_size * 0.5:
                                        merge_target_i = i-1
                                        merge_target_j = k
                                        merge_target_gb = gb
                                        break
                            if merge_target_i >= 0:
                                break
                    if merge_target_i < 0 and i+1 < len(self.grids):
                        for k in range(len(self.grids[i+1])):
                            if len(self.grids[i+1][k]) == 0:
                                continue
                            for gb in self.grid_boundaries:
                                if gb.grid_i == i+1 and gb.grid_j == k and gb.point_count > 0:
                                    y_overlap = min(current_gb.max_y, gb.max_y) - max(current_gb.min_y, gb.min_y)
                                    y_gap = max(current_gb.min_y, gb.min_y) - min(current_gb.max_y, gb.max_y)
                                    if y_overlap > 0 or y_gap < self.base_grid_size * 0.5:
                                        merge_target_i = i+1
                                        merge_target_j = k
                                        merge_target_gb = gb
                                        break
                            if merge_target_i >= 0:
                                break
                    if merge_target_i >= 0 and merge_target_gb is not None:
                        merged_min_x = min(merge_target_gb.min_x, current_gb.min_x)
                        merged_max_x = max(merge_target_gb.max_x, current_gb.max_x)
                        merged_min_y = min(merge_target_gb.min_y, current_gb.min_y)
                        merged_max_y = max(merge_target_gb.max_y, current_gb.max_y)
                        merged_x_size = merged_max_x - merged_min_x
                        merged_y_size = merged_max_y - merged_min_y
                        max_edge = max(merged_x_size, merged_y_size)
                        max_allowed_edge = self.base_grid_size * self.max_aspect_ratio
                        if max_edge <= max_allowed_edge:
                            self.grids[merge_target_i][merge_target_j].extend(self.grids[i][j])
                            merge_target_gb.min_x = merged_min_x
                            merge_target_gb.max_x = merged_max_x
                            merge_target_gb.min_y = merged_min_y
                            merge_target_gb.max_y = merged_max_y
                            merge_target_gb.point_count = len(self.grids[merge_target_i][merge_target_j])
                            self.grids[i][j] = []
                            current_gb.point_count = 0
                            has_merge = True
        # 保存每个网格
        self.total_saved_points = 0
        self.non_empty_grids = 0
        for i in range(len(self.grids)):
            for j in range(len(self.grids[i])):
                grid_size = len(self.grids[i][j])
                for gb in self.grid_boundaries:
                    if gb.grid_i == i and gb.grid_j == j:
                        gb.point_count = grid_size
                        break
                if grid_size > 0:
                    x_size = 0
                    y_size = 0
                    center_x = 0
                    center_y = 0
                    for gb in self.grid_boundaries:
                        if gb.grid_i == i and gb.grid_j == j:
                            x_size = gb.max_x - gb.min_x
                            y_size = gb.max_y - gb.min_y
                            center_x = (gb.min_x + gb.max_x) / 2.0
                            center_y = (gb.min_y + gb.max_y) / 2.0
                            break
                    short_edge = min(x_size, y_size)
                    long_edge = max(x_size, y_size)
                    filename = f"{self.output_dir}grid_{i:03d}_{j:03d}.pcd"
                    # 保存点云
                    cloud = pcl.PointCloud()
                    cloud.from_array(np.array(self.grids[i][j]))
                    # 4. 保存为PCD文件
                    # pcl.save(cloud, filename)
                    
                    utm_x = center_x + self.utm_x0
                    utm_y = center_y + self.utm_y0
                    longitude, latitude = utm_to_lonlat(utm_x, utm_y, self.utm_zone)
                    print(f"Saved Grid [{i},{j}]: {grid_size} points, {latitude:.6f} {longitude:.6f}, x_size={x_size:.2f}m, y_size={y_size:.2f}m")
                    one_grid = [i, j, grid_size, latitude, longitude, x_size, y_size]
                    self.all_grids.append(one_grid)
                    self.total_saved_points += grid_size
                    self.non_empty_grids += 1
                    
        self.export_grids_to_kml(self.output_dir + "grids.kml")
        self.export_batch_sh_for_3dgs(self.output_dir + "batch_scipt_for_get_grid_data_for_3dgs.sh")
        # self.create_grid_folders() #! 新的目录结构， testing

    def world_to_image(self, x, y, img_width=1200, img_height=1200, margin=100):
        max_range = max(self.x_range, self.y_range)
        scale = (img_width - 2 * margin) / max_range
        img_x = int(margin + (x - self.min_x) * scale)
        img_y = int(img_height - margin - (y - self.min_y) * scale)
        return (img_x, img_y)
        
    def visualize(self):
        import random
        img_width = 1200
        img_height = 1200
        margin = 100
        visualization = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255

        # 为每个网格分配一个随机颜色
        grid_colors = {}
        for gb in self.grid_boundaries:
            if gb.point_count == 0:
                continue
            color = tuple([random.randint(0, 255) for _ in range(3)])
            grid_colors[(gb.grid_i, gb.grid_j)] = color

        # 绘制点云
        for gb in self.grid_boundaries:
            if gb.point_count == 0:
                continue
            for grid in self.grids[gb.grid_i][gb.grid_j]:
                pt = self.world_to_image(grid[0], grid[1], img_width, img_height, margin)
                if 0 <= pt[0] < img_width and 0 <= pt[1] < img_height:
                    cv2.circle(visualization, pt, 1, (200, 200, 200), -1)
        # 绘制网格
        for gb in self.grid_boundaries:
            if gb.point_count == 0:
                continue
            top_left = self.world_to_image(gb.min_x, gb.max_y, img_width, img_height, margin)
            bottom_right = self.world_to_image(gb.max_x, gb.min_y, img_width, img_height, margin)
            color = grid_colors.get((gb.grid_i, gb.grid_j), (0, 0, 255))
            cv2.rectangle(visualization, top_left, bottom_right, color, 3)
            text_pos = self.world_to_image((gb.min_x + gb.max_x) / 2, (gb.min_y + gb.max_y) / 2, img_width, img_height, margin)
            cv2.putText(visualization, f"[{gb.grid_i},{gb.grid_j}]", (text_pos[0] - 30, text_pos[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(visualization, f"{gb.point_count} pts", (text_pos[0] - 30, text_pos[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        cv2.putText(visualization, "Grid Division - Top View", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        info = f"Grids: {self.non_empty_grids} | Points: {self.total_saved_points} | Avg: {int(self.total_saved_points/self.non_empty_grids) if self.non_empty_grids > 0 else 0} pts/grid"
        cv2.putText(visualization, info, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        info2 = f"Output: {self.output_dir}"
        cv2.putText(visualization, info2, (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        vis_filename = self.output_dir + "grid_visualization.png"
        cv2.imwrite(vis_filename, visualization)
        print(f"Saved: {vis_filename}")
        # cv2.namedWindow("Grid Division", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("Grid Division", 1200, 1200)
        # cv2.imshow("Grid Division", visualization)
        # print("Press any key to close...")
        # cv2.waitKey(0)


def main():
    config = load_config()
    search_params = config.get('search_params', {})

    output_dir = "/data/exported_roi_data/"
    os.makedirs(output_dir, exist_ok=True)

    # 获取所有session目录
    session_data_dir = '/data/dwm_data/'
    session_dirs = get_all_session_dirs(session_data_dir)
    total_sessions = len(session_dirs)
    print(f'Found {len(session_dirs)} session directories in {session_data_dir}')

    # 获取会话名称列表（用于过滤搜索范围）
    search_session_names = search_params.get('use_session_subdir', [])
    print(f"搜索会话子目录: {search_session_names if search_session_names else '全部会话'}")
    if search_session_names:
        session_dirs = [d for d in session_dirs if os.path.basename(d) in search_session_names]
    else:
        print("未指定搜索会话子目录，将处理所有会话")
    
    all_poses = []
    for idx, session_dir in enumerate(session_dirs, 1):
        # if(idx == 1):
        #     continue
        # if(idx > 4):
        #     break

        print(f'\n\n{"#"*80}')
        poses_dir = os.path.join(session_dir, 'sparse/vehicle_geo_pose')
        print(f'# Session {idx}/{total_sessions}: session_dir = {poses_dir}')

        pose_files = sorted(glob(os.path.join(poses_dir, '*.yaml')))
        for pf in pose_files:
            try:
                pose_data = load_lidar_pose_from_file(pf)
                translation = pose_data.get('translation', None)
                if translation is not None and len(translation) == 3:
                    # translation = [float(translation[1]), float(translation[0]), float(translation[2])]
                    all_poses.append(np.array(translation))
            except Exception as e:
                print(f"[WARN] Failed to read {pf}: {e}")
    print(f"  总计读取到{len(all_poses)}个位姿")

    cloud = save_pose_to_pcd(all_poses, output_dir + "poses_cloud.pcd")

    if cloud is None:
        print("Error: Failed to create point cloud from poses. Exiting.")
        return
    print(f"{all_poses[0][0]}, {all_poses[0][1]}, {all_poses[0][2]}")

    print(f"Original Point Cloud: {cloud.size} points")
    divider = PointCloudGridDivider()
    # grid的基本大小，单位米。最终网格大小会根据点云分布进行调整，但不会超过base_grid_size * max_aspect_ratio。
    divider.base_grid_size = 100.0
    divider.max_aspect_ratio = 1.25
    # 每个网格至少包含的点数，如果一个网格的点数少于这个值，会尝试与相邻网格合并，或者最终被丢弃（不保存）。设置为0表示不进行基于点数的合并。
    divider.min_points_per_grid = 50

    # 没使用
    # divider.use_voxel_filter = True
    # divider.voxel_leaf_size = 0.5
    # divider.max_test_points = 0

    # UTM分区号。
    divider.setUTMZone(51)

    # ! 先设置 输出目录，最终的网格PCD文件和可视化结果都会保存在这个目录下
    divider.setOutputDir(output_dir + "grids/")

    # utm下的位置
    divider.divideAndSave(cloud)
    
    # 可视化划分结果，保存为PNG图片
    divider.visualize()


if __name__ == "__main__":
    main()