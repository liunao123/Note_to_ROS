#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Time
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def modify_and_save_bag(input_bag_file, output_bag_file, time_offset):
    count = 1
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        with rosbag.Bag(output_bag_file, 'w') as outbag:
            # 遍历原始bag文件中的所有消息
            for topic, msg, t in inbag.read_messages():
                timestamp = msg.header.stamp.to_sec()
                # 修改时间戳
                new_time = rospy.Time(count, 0)
                count = count + 1
                print("timestamp:", timestamp)

                if count > 50:
                    break
                
                # 如果消息中包含header，也需要修改header中的时间戳
                # if hasattr(msg, 'header'):
                #     msg.header.stamp = new_time
                
                # # 将修改后的消息写入新的bag文件
                # outbag.write(topic, msg, new_time)

if __name__ == '__main__':
    # ... existing code ...
    
    # 定义输入和输出bag文件路径
    input_bag_file = "/home/bag/nj_xh/xh_vanjee_hk_20250102/v2_vanjee_hk_1.bag"
    output_bag_file = "/home/v2_hk_1106_omm.bag"
    
    # 设置时间偏移量（秒）
    time_offset = 3600  # 例如：向后偏移1小时
    
    # 修改时间戳并保存到新文件
    modify_and_save_bag(input_bag_file, output_bag_file, time_offset)
    
    print("Modified bag file saved as:", output_bag_file)