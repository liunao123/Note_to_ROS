#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import argparse
from rosbag import Bag 
import pyproj
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf.transformations
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# 初始化投影
proj_utm = pyproj.Proj(proj='utm', zone=51, ellps='WGS84', preserve_units=False)

# LiDAR到IMU的外参（在这里直接修改）
LIDAR_TO_IMU_TRANS = [0.0, 0.0, 0.0]  # 平移 [x, y, z] 单位：米
LIDAR_TO_IMU_RPY = [0.0, 0.0, 180.0]     # 旋转 [roll, pitch, yaw] 单位：度

def apply_lidar_imu_transform(imu_position, imu_quaternion):
    """
    将IMU位姿转换到LiDAR坐标系
    Args:
        imu_position: [x, y, z]
        imu_quaternion: [x, y, z, w]
    Returns:
        lidar_position: [x, y, z]
        lidar_quaternion: [x, y, z, w]
    """
    # 构建IMU到世界的变换矩阵
    T_world_imu = tf.transformations.quaternion_matrix(imu_quaternion)
    T_world_imu[0:3, 3] = imu_position
    
    # 构建LiDAR到IMU的变换矩阵
    lidar_to_imu_quat = tf.transformations.quaternion_from_euler(
        LIDAR_TO_IMU_RPY[0] * math.pi / 180.0,
        LIDAR_TO_IMU_RPY[1] * math.pi / 180.0,
        LIDAR_TO_IMU_RPY[2] * math.pi / 180.0
    )
    T_imu_lidar = tf.transformations.quaternion_matrix(lidar_to_imu_quat)
    T_imu_lidar[0:3, 3] = LIDAR_TO_IMU_TRANS
    
    # 求逆得到IMU到LiDAR的变换
    T_lidar_imu = np.linalg.inv(T_imu_lidar)
    
    # 计算世界到LiDAR的变换: T_world_lidar = T_world_imu * T_lidar_imu
    T_world_lidar = np.dot(T_world_imu, T_lidar_imu)
    
    # 提取位置和姿态
    lidar_position = T_world_lidar[0:3, 3].tolist()
    lidar_quaternion = tf.transformations.quaternion_from_matrix(T_world_lidar).tolist()
    
    return lidar_position, lidar_quaternion

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
    print(f"LiDAR到IMU外参: 平移={LIDAR_TO_IMU_TRANS}, 旋转RPY(度)={LIDAR_TO_IMU_RPY}")
    files = get_files_list(args.input, args.select)
    
    # 用于存储第一个位置作为基准
    first_position = None
    
    with Bag(args.output, "w" ) as o:
        start = time.perf_counter()
        for i in range(len(files)):
            show_process_bar(len(files), i+1, start)
            with Bag(files[i], "r") as ib:
                for topic, msg, t in ib:
                    if 'cam' in topic:
                        continue

                    # 处理/chcnav/devpvt，转换为Odometry
                    if topic == "/chcnav/devpvt":
                        odom_msg = Odometry()
                        odom_msg.header = msg.header
                        odom_msg.header.frame_id = "map"
                        odom_msg.child_frame_id = "PandarSwiftOfficial"
                        
                        # 将经纬度转换为UTM坐标
                        x, y = proj_utm(msg.longitude, msg.latitude)
                        z = msg.altitude
                        
                        # 记录第一个位置作为基准
                        if first_position is None:
                            first_position = {'x': x, 'y': y, 'z': z}
                            print(f"\n基准位置设置为: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                        
                        # 减去基准位置
                        odom_msg.pose.pose.position.x = x - first_position['x']
                        odom_msg.pose.pose.position.y = y - first_position['y']
                        odom_msg.pose.pose.position.z = z - first_position['z']
                        
                        # 设置姿态（roll, pitch, yaw单位为度，需要转换为弧度）
                        roll = msg.roll * math.pi / 180.0
                        pitch = msg.pitch * math.pi / 180.0
                        yaw = msg.yaw * math.pi / 180.0
                        
                        # 转换为四元数 二者等价
                        # quaternion = R.from_euler('XYZ', [roll, pitch, yaw], degrees=False).as_quat()
                        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                        odom_msg.pose.pose.orientation.x = quaternion[0]
                        odom_msg.pose.pose.orientation.y = quaternion[1]
                        odom_msg.pose.pose.orientation.z = quaternion[2]
                        odom_msg.pose.pose.orientation.w = quaternion[3]

                        imu_position = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]
                        imu_orientation = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
                        lidar_position, lidar_orientation = apply_lidar_imu_transform(imu_position, imu_orientation)
                        
                        odom_msg.pose.pose.position.x = lidar_position[0]
                        odom_msg.pose.pose.position.y = lidar_position[1]
                        odom_msg.pose.pose.position.z = lidar_position[2]
                        odom_msg.pose.pose.orientation.x = lidar_orientation[0]
                        odom_msg.pose.pose.orientation.y = lidar_orientation[1]
                        odom_msg.pose.pose.orientation.z = lidar_orientation[2]
                        odom_msg.pose.pose.orientation.w = lidar_orientation[3]

                        # 创建TF消息：odom -> lidar
                        tf_msg = TFMessage()
                        transform = TransformStamped()
                        transform.header.stamp = msg.header.stamp
                        transform.header.frame_id = "map"
                        transform.child_frame_id = "PandarSwiftOfficial"
                        transform.transform.translation.x = lidar_position[0]
                        transform.transform.translation.y = lidar_position[1]
                        transform.transform.translation.z = lidar_position[2]
                        transform.transform.rotation.x = lidar_orientation[0]
                        transform.transform.rotation.y = lidar_orientation[1]
                        transform.transform.rotation.z = lidar_orientation[2]
                        transform.transform.rotation.w = lidar_orientation[3]
                        tf_msg.transforms.append(transform)
                        
                        # 写入TF消息
                        o.write("/tf", tf_msg, t)

                        
                        # 设置速度（enu_velocity是Vector3类型：东、北、天）
                        odom_msg.twist.twist.linear.x = msg.enu_velocity.x  # 东
                        odom_msg.twist.twist.linear.y = msg.enu_velocity.y  # 北
                        odom_msg.twist.twist.linear.z = msg.enu_velocity.z  # 天
                        
                        # 设置角速度（vehicle_angular_velocity单位为度/秒，需要转换为弧度/秒）
                        odom_msg.twist.twist.angular.x = msg.vehicle_angular_velocity.x * math.pi / 180.0
                        odom_msg.twist.twist.angular.y = msg.vehicle_angular_velocity.y * math.pi / 180.0
                        odom_msg.twist.twist.angular.z = msg.vehicle_angular_velocity.z * math.pi / 180.0
                        
                        # 写入转换后的Odometry消息
                        o.write("/chcnav/odom", odom_msg, t)
                        continue
                    
                    # 其他topic直接写入
                    o.write(topic, msg, t)
                        
            show_process_bar(len(files), i+1, start)

    print("\n\n 77   end merge bags.")
    
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
