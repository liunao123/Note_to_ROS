#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag

# 输入的bag文件路径
bag_file = "/home/direct_l_v_calibrate_data_avia/0.bag"

# 要处理的图像topic
topic_to_process = "/hk_camera/image_color"

# 保留的帧数
num_frames_to_keep = 3

# 输出的处理结果
processed_data = []

# 读取输入的bag文件
with rosbag.Bag(bag_file, 'r') as input_bag:
    # 帧计数器
    frame_count = 0
    
    # 遍历所有消息
    for topic, msg, t in input_bag.read_messages(topics=[topic_to_process]):
        # 检查当前消息的topic是否是要处理的topic
        if topic == topic_to_process:
            # 增加帧计数器
            frame_count += 1
            
            # 处理前 num_frames_to_keep 帧数据
            if frame_count <= num_frames_to_keep:
                # 在这里添加您的处理逻辑
                # 示例：将消息的数据添加到处理结果列表中
                processed_data.append(msg)

    for topic, msg, t in input_bag.read_messages():
        # 检查当前消息的topic是否在要保留的列表中
        if topic in topics_to_keep:
            # 写入消息到输出的bag文件
            output_bag.write(topic, msg, t)


        # 如果已经保留了 num_frames_to_keep 帧数据，则退出循环
        if frame_count > num_frames_to_keep:
            break

# 打印处理结果
print("处理结果:")
for data in processed_data:
    print(data)