#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CameraInfo
import numpy as np

def publish_camera_info():
    # 初始化ROS节点
    rospy.init_node('camera_info_publisher', anonymous=True)
    
    # 创建发布者，发布camera_info话题
    pub = rospy.Publisher('/hk_camera/camera_info', CameraInfo, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(1)  # 10Hz
    
    # 创建CameraInfo消息
    camera_info_msg = CameraInfo()
    
    # 设置消息头
    camera_info_msg.header.frame_id = "hk_camera"
    camera_info_msg.distortion_model = "plumb_bob"
    
    # 设置图像尺寸
    camera_info_msg.height = 1024
    camera_info_msg.width = 1224

    # 设置相机内参矩阵 (示例值，需要根据实际相机标定结果修改)
    camera_info_msg.K = [
        1252.56725751, 0.0, 625.67388127,
        0.0, 1252.63595805, 524.95133965,
        0.0, 0.0, 1.0
    ]

    # 设置畸变参数 (示例值，需要根据实际相机标定结果修改)
    camera_info_msg.D = [-0.11332013 , 0.18298739 , 0.00070469 , -0.00056838, 0.0]
    
    # 设置投影矩阵
    camera_info_msg.P = [
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    ]
    
    # 设置旋转矩阵
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    
    while not rospy.is_shutdown():
        # 更新时间戳
        camera_info_msg.header.stamp = rospy.Time.now()
        
        # 发布消息
        pub.publish(camera_info_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass