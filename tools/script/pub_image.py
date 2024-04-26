#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_images(folder_path, topic):
    # 初始化ROS节点
    rospy.init_node('image_publisher', anonymous=True)

    # 创建图像发布者
    image_publisher = rospy.Publisher(topic, Image, queue_size=10)

    # 创建CvBridge对象
    bridge = CvBridge()

    # 获取文件夹中的所有图像文件
    image_files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

    # 遍历所有图像文件
    for file_name in image_files:
        # 读取图像文件
        image_path = os.path.join(folder_path, file_name)
        image = cv2.imread(image_path)

        if image is not None:
            # 将OpenCV图像转换为ROS图像消息
            ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # 发布图像消息
            image_publisher.publish(ros_image)
            rospy.loginfo("Published image: %s", file_name)
        else:
            rospy.logwarn("Failed to read image: %s", file_name)

        # 延时一段时间（例如0.1秒）
        rospy.sleep(0.2)

if __name__ == '__main__':
    folder_path = "/home/liunao/qt/liejian/cab_liejian/camera_in/3"  # 替换为你的图像文件夹路径
    topic = "/image"  # 替换为你要发布的topic名称
    try:
        publish_images(folder_path, topic)
    except rospy.ROSInterruptException:
        pass