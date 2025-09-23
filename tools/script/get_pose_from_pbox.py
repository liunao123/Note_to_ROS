#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import argparse
from rosbag import Bag 
import pyproj
from geometry_msgs.msg import PoseStamped
import tf.transformations
import numpy as np
import statistics
import matplotlib.pyplot as plt
import math
from time import sleep
# 初始化投影
proj_utm = pyproj.Proj(proj='utm', zone=51, ellps='WGS84', preserve_units=False)

import numpy as np
from pyproj import Proj, Transformer


def llh_to_enu(lat, lon, alt, lat0 = 31.428355089295906, lon0 = 120.6220900705886, alt0 = 13.683439254760742):
# def llh_to_enu(lat, lon, alt, lat0 = 31.42531422337482, lon0 = 120.62295062399883, alt0 = 12.800246238708496):
    """
    将经纬高(LLH)转换为东北天(ENU)坐标系
    
    参数:
    lat, lon, alt: 目标点的纬度(度), 经度(度), 高度(m)
    lat0, lon0, alt0: 本地坐标系原点的纬度(度), 经度(度), 高度(m)
    
    返回:
    e, n, u: 东、北、天方向坐标(m)
    """
    # WGS84椭球参数
    a = 6378137.0  # 地球长半轴(m)
    f = 1 / 298.257223563  # 扁率
    b = a * (1 - f)  # 短半轴
    e_sq = 1 - (b * b) / (a * a)  # 第一偏心率的平方
    
    # 将经纬度转换为弧度
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)
    
    # 计算基准点的地心坐标
    N0 = a / np.sqrt(1 - e_sq * np.sin(lat0_rad) * np.sin(lat0_rad))
    x0 = (N0 + alt0) * np.cos(lat0_rad) * np.cos(lon0_rad)
    y0 = (N0 + alt0) * np.cos(lat0_rad) * np.sin(lon0_rad)
    z0 = (N0 * (1 - e_sq) + alt0) * np.sin(lat0_rad)
    
    # 计算目标点的地心坐标
    N = a / np.sqrt(1 - e_sq * np.sin(lat_rad) * np.sin(lat_rad))
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e_sq) + alt) * np.sin(lat_rad)
    
    # 计算ENU坐标
    dx = x - x0
    dy = y - y0
    dz = z - z0
    
    # 旋转矩阵：ECEF到ENU
    sin_lat0 = np.sin(lat0_rad)
    cos_lat0 = np.cos(lat0_rad)
    sin_lon0 = np.sin(lon0_rad)
    cos_lon0 = np.cos(lon0_rad)
    
    e = -sin_lon0 * dx + cos_lon0 * dy
    n = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    u = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    
    return e, n, u

# 更简洁的版本
def enu_to_transform_matrix_compact(e, n, u, roll, pitch, yaw):
    """
    简洁版本的ENU到变换矩阵转换
    """
    # 创建平移部分
    transform = np.eye(4)
    transform[:3, 3] = [e, n, u]
    
    # 计算旋转矩阵的各个元素
    cy, sy = np.cos(yaw), np.sin(yaw)      # Z轴旋转
    cp, sp = np.cos(pitch), np.sin(pitch)  # Y轴旋转  
    cr, sr = np.cos(roll), np.sin(roll)    # X轴旋转
    
    # 直接构造旋转矩阵 (ZYX顺序)
    rotation = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, 0],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, 0],
        [-sp,   cp*sr,            cp*cr,            0],
        [0,     0,                0,                1]
    ])
    
    # 组合变换矩阵
    transform = transform @ rotation
    return transform

def get_files_list(arg_input, arg_select):
    # 只读取指定文件夹下的所有文件（不区分扩展名），按文件名排序
    if not os.path.isdir(arg_input):
        raise ValueError(f"{arg_input} 不是一个有效的文件夹路径")
    files = [os.path.join(arg_input, f) for f in os.listdir(arg_input) if os.path.isfile(os.path.join(arg_input, f))]
    files.sort()  # 按文件名排序
    if arg_select:
        files = select_bags(files)
    print("bags list:")
    for i, f in enumerate(files):
        print(f"{i} . {f}")
    return files
def select_bags(files):
    for i in range(len(files)):
        print(str(i) + "." + files[i])
    print("Please input bag numbers to merge. split by , or space", end=":")
    s_b = input()
    s_b_list = []
    if "," in s_b:
        s_b_list = s_b.split(",")
    elif " " in s_b:
        s_b_list = s_b.split(" ")
    files = [files[int(x)] for x in s_b_list]
    return files
def show_process_bar(total, i, start):
    a = "*" * i
    b = "." * (total - i)
    c = (i / total) * 100
    dur = time.perf_counter() - start
    print("\r{:^3.0f}%[{}->{}]{:.2f}s".format(c,a,b,dur), end="")
def merge_bags(args):
    print("start merge bags.")
    files = get_files_list(args.input, args.select)

    count = 0
    first_pose = np.eye(4)
    
    with Bag(args.output, "w" ) as o:
    # with open("poses.txt", "w") as f:
        start = time.perf_counter()
        for i in range(len(files)):
        # for i in range(3):
            show_process_bar(len(files), i+1, start)
            with open('/home/tyjt/Desktop/drivewise_v101/data/park/pose.txt', 'w', encoding='utf-8') as f1:
                # with Bag(files[i], "r") as ib:
                bag_file = "/mnt/nvme0n1p2/data/0904/20250904_tj_200s.bag"
                last_yaw = 0
                with Bag(bag_file, "r") as ib:
                    for topic, msg, t in ib:
                        if topic == "/chcnav/devpvt" :
                            # if  msg.stat[1] >= 2 and msg.stat[1] <= 5: 
                            # if  msg.stat[1] == 4 or msg.stat[1] == 8 : # 72136 for all 
                            #     pass
                            # else:
                            #     continue
                            # print(msg.latitude, msg.longitude, msg.altitude)
                            if abs( msg.yaw - last_yaw ) > 2.0:
                                print(msg.roll, msg.pitch, msg.yaw, last_yaw)
                                last_yaw = msg.yaw 

                            x, y = proj_utm(msg.longitude, msg.latitude)
                            # 欧拉角转四元数
                            # quat = tf.transformations.quaternion_from_euler(0, 0, ( - msg.heading2  ) * 3.1415926 / 180.0 )
                            quat = tf.transformations.quaternion_from_euler(msg.roll * 3.1415926 / 180.0, msg.pitch * 3.1415926 / 180.0, -1.0 *( msg.yaw  ) * 3.1415926 / 180.0 ,'szyx')                      # 构造PoseStamped消息
                            pose = PoseStamped()
                            pose.header = msg.header
                            pose.pose.position.x = x 
                            pose.pose.position.y = y 
                            pose.pose.position.z = msg.altitude
                            pose.pose.orientation.x = quat[0]
                            pose.pose.orientation.y = quat[1]
                            pose.pose.orientation.z = quat[2]
                            pose.pose.orientation.w = quat[3]
                            count += 1
                            f1.write("{:d} {:.3f} {:.3f} {:.3f} {:.3f} {:.4f} {:.4f} {:.4f} {:.4f}".format(count, msg.header.stamp.to_sec(),  x,  y, msg.altitude, quat[0], quat[1], quat[2], quat[3]))
                            continue
    
                            # o.write(topic, msg, t)
                            # print(pose.pose.position.x,  pose.pose.position.y)
                            # if pose.pose.position.y < -533.60 and pose.pose.position.y > -533.620 and pose.pose.position.x < 1010.01 and pose.pose.position.x > 1005.02:
                            # # if pose.pose.position.y < -170.0 and pose.pose.position.y > -172.0 and pose.pose.position.x < 965.0 and pose.pose.position.x > 960.0:
                            #    data.append( msg.yaw )
                            #    print( msg.yaw )
                            # o.write("/apollo/localization/pose", pose, t)
    
                            e,n,u = llh_to_enu( msg.latitude, msg.longitude, msg.altitude )
                            now_pose = enu_to_transform_matrix_compact(e, n, u, msg.roll* 3.1415926 / 180.0, msg.pitch* 3.1415926 / 180.0,  ( msg.yaw - 90.0 )* 3.1415926 / 180.0 )
                            # now_pose = enu_to_transform_matrix_compact(e, n, u, msg.roll* 3.1415926 / 180.0, msg.pitch* 3.1415926 / 180.0,  ( msg.yaw  )* 3.1415926 / 180.0 )
                            
                            if(count == 0):
                                first_pose = now_pose
                            else:
                                inv_first_pose = np.linalg.inv(first_pose) @ now_pose
                                # print( inv_first_pose[:, -1] )
                                print("{:.3f} {:.3f} {:.3f} {:.2f}".format(inv_first_pose[0, 3], inv_first_pose[1, 3], inv_first_pose[2, 3], inv_first_pose[3, 3]))
                            count += 1
                            sleep(0.01)
    
                        # if topic == "/rslidar_points" :
                        #     o.write("/apollo/sensor/velodyne64/compensator/PointCloud2", msg, t)
                            

            show_process_bar(len(files), i+1, start)
    print("121   end merge bags.")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge one or more bag files to one file.")
    parser.add_argument("-o", "--output", help="Output bag's file path.",
                        default="output.bag", required=True)
    parser.add_argument("-i", "--input", help="Input files bags name or path, split by , .",
                        required=True)
    parser.add_argument("-s", "--select", help="Select bags to merge.",
                        default=False)
    parser.add_argument("-v", "--verbose", help="Show the verbose msg.")
    args = parser.parse_args()
    merge_bags(args)