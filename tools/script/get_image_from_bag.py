#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
image_topic = '/dh_camera/image_color'

def save_img_from_bag( bag_file , img_file):
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    count = 1
    for topic, msg, t in bag.read_messages(topics=[ image_topic]):
       # cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        timestamp = msg.header.stamp.to_sec()
        # 构建文件名，将时间戳作为文件名的一部分
        # img_file = "/home/liunao/Kalibr/v1_hall_bag_extrisics/v1_%s.png" % str(count)
        # img_file = file_path + "3.png"

        count += 1
        # img_file = "/home/dlvc_data/vel2camera_vanjee_hk/vanjee_hk_xr_yf/3.jpg" # % str(count-2)
        if count > 3  and count < 5:
        # if 1:
        # if  timestamp > 1609059153.0 and timestamp < 1609059155.02 :
            cv2.imwrite(img_file, cv_img)
            print("Image saved as ", img_file , timestamp)
            break
    bag.close()

    return

if __name__ == '__main__':
    file_path = "/home/liunao/Kalibr/lx100_dh/lx100_dh_rs_20241217/bag/"
    print( file_path )
    # bag_file = "/opt/csg/slam/navs/v1_20241118/bag/33.bag"
    # save_img_from_bag( bag_file )

    for i in range(9):
        bag_file =  file_path + str(i) + ".bag"
        img_file = file_path + str(i) + ".png"
        print( bag_file )
        save_img_from_bag(bag_file , img_file)

    print("Extracted image DONE")
