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

import rosbag
import pcl
import numpy as np
import os
from sensor_msgs.msg import PointCloud2
from tqdm import tqdm
import shutil


def convert_pointcloud2_to_pcl(pc2_msg):
    """
    将PointCloud2消息转换为PCL点云对象
    """
    # 提取点云数据
    points = []
    for i in range(0 , pc2_msg.width * pc2_msg.height , 1 ):
        # 计算每个点的字节偏移量
        point_offset = i * pc2_msg.point_step
        
        # 提取xyz坐标（假设前12个字节是xyz）
        x = np.frombuffer(pc2_msg.data[point_offset:point_offset+4], dtype=np.float32)[0]
        y = np.frombuffer(pc2_msg.data[point_offset+4:point_offset+8], dtype=np.float32)[0]
        z = np.frombuffer(pc2_msg.data[point_offset+8:point_offset+12], dtype=np.float32)[0]
        
        # 提取强度值（可能需要根据实际字段偏移量调整）
        intensity_offset = 16  # 通常强度值在16字节偏移处
        intensity = np.frombuffer(pc2_msg.data[point_offset+intensity_offset:point_offset+intensity_offset+4], dtype=np.float32)[0]

        if abs(x) < 2.5 and abs(y) < 2.5:
            continue;
        
        # 过滤无效点
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or
                np.isinf(x) or np.isinf(y) or np.isinf(z)):
            points.append([x, y, z, intensity])
    
    if len(points) == 0:
        return None
    
    # 转换为numpy数组
    points = np.array(points, dtype=np.float32)
    
    # 创建带强度值的PCL点云
    cloud = pcl.PointCloud_PointXYZI()
    cloud.from_array(points)
    
    return cloud

 
def get_files_list(arg_input ):
    # 只读取指定文件夹下的所有文件（不区分扩展名），按文件名排序
    if not os.path.isdir(arg_input):
        raise ValueError(f"{arg_input} 不是一个有效的文件夹路径")
    files = [os.path.join(arg_input, f) for f in os.listdir(arg_input) if os.path.isfile(os.path.join(arg_input, f))]
    files.sort()  # 按文件名排序
    for i, f in enumerate(files):
        print(f"{i} . {f}")
    return files
 
def show_process_bar(total, i, start):
    a = "*" * i
    b = "." * (total - i)
    c = (i / total) * 100
    dur = time.perf_counter() - start
    print("\r{:^3.0f}%[{}->{}]{:.2f}s".format(c,a,b,dur), end="")
def merge_bags(args):
    print("start merge bags.")
    files = get_files_list( args.input )

    count_pose = 0
    count_lidar = 0
    first_pose = np.eye(4)
    # output_dir = '/mnt/nvme0n1p2/data/0909_pcd/'
    output_dir =  args.output
    
    # shutil.rmtree(output_dir) #将整个文件夹删除
    os.makedirs(output_dir, exist_ok=True)

    unvaild_pose_time = 0.0

    start = time.perf_counter()
    for i in range(len(files)):
        show_process_bar(len(files), i+1, start)
        with open(output_dir + 'pose.txt', 'w', encoding='utf-8') as f1,\
             open(output_dir + 'pcd_timestamp.txt', 'w', encoding='utf-8') as f2:
            with Bag(files[i], "r") as ib:
                for topic, msg, t in ib:
                    if topic == "/chcnav/devpvt" :
                        # if  msg.stat[1] != 4:
                        #     unvaild_pose_time = msg.header.stamp.to_sec()
                        #     continue
                        # print(msg.stat)
                        # print(msg.position_stdev)
                        # print(msg.euler_stdev)
                        # print( )
                        # if msg.euler_stdev[2] > 0.6:
                        # continue
                        
                        # print(msg.latitude, msg.longitude, msg.altitude)
                        x, y = proj_utm(msg.longitude, msg.latitude)
                        # 欧拉角转四元数
                        # ok
                        # quat = tf.transformations.quaternion_from_euler(0, 0, -1.0 * ( msg.heading2 -90.0  ) *  math.pi  / 180.0 )

                        quat = tf.transformations.quaternion_from_euler(msg.roll *  math.pi  / 180.0, msg.pitch *  math.pi  / 180.0,  ( -1.0 * msg.yaw ) *  math.pi  / 180.0  )                      # 构造PoseStamped消息
                        # quat = tf.transformations.quaternion_from_euler(msg.roll * math.pi / 180.0, msg.pitch *  math.pi  / 180.0, -1.0 * ( msg.heading2 - 90.0  ) *  math.pi  / 180.0  )                      # 构造PoseStamped消息
                        count_pose += 1
                        f1.write("{:d} {:.3f} {:.4f} {:.4f} {:.4f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(count_pose, msg.header.stamp.to_sec(),  x,  y, msg.altitude, quat[0], quat[1], quat[2], quat[3]))
                        # f1.write("{:.3f} {:.4f} {:.4f} {:.4f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format( msg.header.stamp.to_sec(),  x,  y, msg.altitude, quat[0], quat[1], quat[2], quat[3]))
                        # continue

                    if topic == "/rslidar_points":
                        timestamp = msg.header.stamp.to_nsec()
                        # if abs( msg.header.stamp.to_sec() - unvaild_pose_time ) < 0.50 :
                        #     print("skip lidar frame for unvaild pose ", msg.header.stamp.to_sec() , unvaild_pose_time  )
                        #     continue
                        count_lidar += 1
                        f2.write(f"{count_lidar} {msg.header.stamp.to_sec()}\n")
                        # continue
                        cloud = convert_pointcloud2_to_pcl(msg)
                        # 生成文件名
                        filename = os.path.join(output_dir +  f"{count_lidar}.pcd")
                        print(f"Saving point cloud to {filename}")
                        # 保存为PCD文件
                        pcl.save(cloud, filename, binary=False)  # binary=False 保存为ASCII格式
                    
                    # print(f"Saving point cloud to {count_lidar}   {count_pose}")
                    # if count_lidar > 500 and count_pose > 5100 :
                    #     break
                        
        show_process_bar(len(files), i+1, start)

    print("121   end merge bags.")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge one or more bag files to one file.")
    parser.add_argument("-o", "--output", help="Output bag's file path.",
                        default="output.bag", required=True)
    parser.add_argument("-i", "--input", help="Input files bags name or path, split by , .",
                        required=True)
    args = parser.parse_args()
    merge_bags(args)