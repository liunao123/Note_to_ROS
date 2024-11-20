#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Time
import rosbag
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from livox_ros_driver.msg import CustomMsg
import numpy as np
import sensor_msgs.point_cloud2 as pc2

def custom_msg_to_point_cloud2(custom_msg):
    # Extract points from CustomMsg
    points = []
    for point in custom_msg.points:
        x, y, z = point.x, point.y, point.z
        intensity = point.reflectivity
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

def modify_and_save_bag(input_bag_file, output_bag_file, time_offset):
    count = 1
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        with rosbag.Bag(output_bag_file, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                new_time = rospy.Time(count, 0)
                count = count + 1
                print("Processing message:", count, "Topic:", topic)

                # Convert Livox CustomMsg to PointCloud2
                if "livox_ros_driver/CustomMsg" in msg._type:
                    msg = custom_msg_to_point_cloud2(msg)
                    # Update topic name for the converted message
                    topic = topic + "/pointcloud2"

                if hasattr(msg, 'header'):
                    msg.header.stamp = new_time
                
                outbag.write(topic, msg, new_time)

# ... rest of the code remains the same ... 