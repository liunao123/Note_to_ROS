#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        # 创建一个CvBridge对象
        bridge = CvBridge()

        # 将ROS图像消息转换为OpenCV图像格式
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 获取图像消息的时间戳
        timestamp = msg.header.stamp.to_sec()
        # if timestamp > 1678262970.963 and timestamp < 1678262972.963 :
        if 1 :
            # 将时间戳转换为可读格式
    
            # 构建文件名，将时间戳作为文件名的一部分
            # filename = f"image_{timestamp_str}.png"
            # filename = "/home/bag/NTU_data/eee_01/r2_feature/eee_01_%s.png" % str(timestamp)
            filename = "/home/0%s.jpg" % str(timestamp)
    
            # 保存图像到文件
            cv2.imwrite(filename, cv_image)
            print("Image saved as", filename)

    except cv2.error as e:
        print(e)

def image_subscriber():
    # 初始化ROS节点
    rospy.init_node('image_subscriber', anonymous=True)

    # 创建一个订阅者，订阅名为"/image_topic"的图像topic
    # rospy.Subscriber("/feature_tracker/feature_img", Image, image_callback)
    rospy.Subscriber("/cam0/image_raw", Image, image_callback)
    # rospy.Subscriber("/camera/image_color", Image, image_callback)
    # rospy.Subscriber("/rgb_img", Image, image_callback)
    # rospy.Subscriber("/left/image_raw", Image, image_callback)

    # 循环等待消息
    rospy.spin()

if __name__ == '__main__':
    image_subscriber()
