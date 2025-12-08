#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import pyproj
import struct
import os


def llh_to_utm(lat, lon, utm_zone=51):
    """
    将经纬度转换为UTM坐标
    """
    proj_utm = pyproj.Proj(proj='utm', zone=utm_zone,
                           ellps='WGS84', preserve_units=False)
    x, y = proj_utm(lon, lat)
    return x, y


def write_pcd_ascii(points, filename):
    """
    将点云数据写入ASCII格式的PCD文件
    """
    num_points = len(points)

    # PCD文件头
    header = f"""# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""

    with open(filename, 'w') as f:
        f.write(header)
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

    print(f"成功保存 {num_points} 个点到文件: {filename}")


def read_rtk_points_and_convert(input_file, output_file ):
    """
    读取RTK点文件并转换为TUM坐标系下的PCD文件
    
    Args:
        input_file: RTK点文件路径
        output_file: 输出PCD文件路径
        reference_lat: 参考点纬度
        reference_lon: 参考点经度  
        reference_alt: 参考点高度
    """
    points = []

    # 计算参考点的UTM坐标
    # ref_x, ref_y = llh_to_utm(reference_lat, reference_lon)
    ref_x = 662334.92633696645
    ref_y = 4873532.7935474264
    reference_alt = 0
    print(f"参考点 UTM 坐标: ({ref_x:.3f}, {ref_y:.3f})")

    try:
        with open(input_file, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue

                try:
                    # 解析纬度、经度、高度（数据组织为：纬度 经度 高度）
                    parts = line.split()
                    if len(parts) >= 3:
                        lat = float(parts[0])  # 纬度
                        lon = float(parts[1])  # 经度
                        alt = float(parts[2])  # 高度
                        print(f" LLA : ({lat:.9f}, {lon:.9f}, {alt:.3f})")

                        # 转换为UTM坐标
                        x_utm, y_utm = llh_to_utm(lat, lon)

                        # 转换为相对于参考点的坐标 (TUM坐标系)
                        x_tum = x_utm - ref_x
                        y_tum = y_utm - ref_y
                        z_tum = alt  # 相对高度

                        points.append([x_tum, y_tum, z_tum])

                    else:
                        print(f"警告: 第{line_num}行格式不正确: {line}")

                except ValueError as e:
                    print(f"警告: 第{line_num}行数据解析错误: {line}, 错误: {e}")

    except FileNotFoundError:
        print(f"错误: 找不到文件 {input_file}")
        return
    except Exception as e:
        print(f"错误: 读取文件时出现异常: {e}")
        return

    if not points:
        print("错误: 没有有效的点数据")
        return

    print(f"成功读取 {len(points)} 个RTK点")

    # 转换为numpy数组
    points = np.array(points)

    # 打印统计信息
    print(f"X 范围: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] m")
    print(f"Y 范围: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] m")
    print(f"Z 范围: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}] m")

    # 保存为PCD文件
    write_pcd_ascii(points, output_file)


def main():
    # 文件路径
    input_file = "/mnt/nvme0n1p2/data/qcsl.rtk"
    output_file = "/home/tyjt/Desktop/Note_to_ROS/tools/rtk_points_nongan.pcd"

    # 参考点坐标 (你可以根据需要修改)
    reference_lat = 0   # 参考纬度
    reference_lon = 0   # 参考经度
    reference_alt = 0   # 参考高度

    print("RTK点转PCD程序开始运行...")
    print(f"输入文件: {input_file}")
    print(f"输出文件: {output_file}")
    print(f"参考点: 纬度={reference_lat}, 经度={reference_lon}, 高度={reference_alt}")

    # 执行转换
    read_rtk_points_and_convert(
        input_file=input_file,
        output_file=output_file 
    )

    print("程序执行完成!")


if __name__ == "__main__":
    main()
