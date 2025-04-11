#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Time
import rosbag
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from livox_ros_driver.msg import CustomMsg
import numpy as np
import sensor_msgs.point_cloud2 as pc2

def get_less_intensity_pts(pts_msg):
    points = []
    # for point in pc2.read_points(pts_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
    for point in pc2.read_points(pts_msg, field_names=("x", "y", "z", "r", "g", "b"), skip_nans=True):
        # x, y, z, intensity = point
        x, y, z, r, g, b = point
        # if intensity < 100:   # keep intensity less than 100
        if z < 1:   # keep intensity less than 100
            # points.append([x, y, z, intensity])
            points.append( [x, y, z, r, g, b] )


    print("points.size : ", len(points))

    # Convert to numpy array
    points_array = np.array(points, dtype=np.float32)

    # Create PointCloud2 message
    header = pts_msg.header
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1),
    ]
    
    pc2_msg = pc2.create_cloud(header, fields, points_array)
    return pc2_msg



def custom_msg_to_point_cloud2(custom_msg):
    # Extract points from CustomMsg
    points = []
    for point in custom_msg.points:
        x, y, z = point.x, point.y, point.z
        intensity = point.reflectivity
        if x > 1.0 and intensity < 120:
            points.append([x, y, z, intensity])

    # Convert to numpy array
    points_array = np.array(points, dtype=np.float32)

    # Create PointCloud2 message
    header = custom_msg.header
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1),
    ]
    
    pc2_msg = pc2.create_cloud(header, fields, points_array)
    return pc2_msg

def modify_and_save_bag(input_bag_file, output_bag_file):
    count = 0
    image_count = 0
    total_images = 0
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        with rosbag.Bag(output_bag_file, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                count += 1
                if count < 10:
                    pass
                # print("Processing message:", count, "Topic:", topic)
                # Convert Livox CustomMsg to PointCloud2
                if "livox_ros_driver/CustomMsg" in msg._type:
                    msg = custom_msg_to_point_cloud2(msg)
                    # Update topic name for the converted message
                    topic ="/livox_pc2"

                if "Image" in msg._type:
                    image_count += 1
                    if (5 <= image_count <= 10) :
                        pass
                    else:
                        continue

                if "sensor_msgs/PointCloud2" in msg._type:
                    msg = get_less_intensity_pts(msg)


                outbag.write(topic, msg)

if __name__ == '__main__':
    # ... existing code ...
    
    # 定义输入和输出bag文件路径
    path_bag = "/opt/csg/slam/navs/test_d435i/1.bag"
    out_path = "/opt/csg/slam/navs/test_d435i/2.bag"
    modify_and_save_bag(path_bag, out_path)

    # bag_index = 0

    # for bag_index in range(12):
    #     # if bag_index != 6:
    #     #     continue
    #     # Format bag index with leading zero (00, 01, 02, etc.)
    #     bag_num = str(bag_index)
    #     input_bag_file = path_bag + bag_num    + ".bag"
    #     out_bag_file = out_path + bag_num   + ".bag"
        
    #     print("Modified bag file saved as:", input_bag_file)
    #     print("Modified bag file saved as:", out_bag_file)

    #     # 修改时间戳并保存到新文件
    #     modify_and_save_bag(input_bag_file, out_bag_file)
        
    # print("Modified bag DONE:" )