#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Time
import rosbag
import math
from chcnav.msg import hcinspvatzcb # 这就是你的自定义消息
from sensor_msgs.msg import Imu

# heading 转换为 yaw
# by ok 20250917
def convertHeadingToYaw( heading ):
    yaw = -1.0 * heading
    if yaw > 180.0:
        yaw = yaw - 180.0
    elif yaw < -180.0:
        yaw = yaw + 360.0

    return yaw
 

save_topics = {"/rslidar_points", "/gps/fix", "/gps/gps", "/bynav/heading2", "/odom", }  

def modify_and_save_bag(input_bag_file, output_bag_file):
    with rosbag.Bag(input_bag_file, 'r') as inbag:
        with rosbag.Bag(output_bag_file, 'w') as outbag:
            # 遍历原始bag文件中的所有消息
            count = 0
            for topic, msg, t in inbag.read_messages():
                if topic == '/bynav/inspvax':
                    # print("timestamp:", t.to_sec())
                    # print("ins_status:",  msg.ins_status )
                    # print("ins_status:",  msg.pos_type )
                    # if msg.ins_status.status == 3 and msg.pos_type.type == 56 :
                    #     count = count + 1

                    hc_msg = hcinspvatzcb()
                    hc_msg.header = msg.header
                    hc_msg.week = msg.nov_header.gps_week_number
                    hc_msg.second = msg.nov_header.gps_week_milliseconds / 1000.0

                    hc_msg.longitude = msg.longitude
                    hc_msg.latitude = msg.latitude
                    hc_msg.altitude = msg.height + msg.undulation  # 转换为 椭球高 = 海拔高+高程异常 

                    hc_msg.roll = msg.roll
                    hc_msg.pitch = msg.pitch
                    hc_msg.yaw = convertHeadingToYaw( msg.azimuth )

                    hc_msg.speed =  math.sqrt(msg.north_velocity * msg.north_velocity + msg.east_velocity * msg.east_velocity + msg.up_velocity * msg.up_velocity )  
                    outbag.write("/chcnav/devpvt", hc_msg, Time.from_sec( t.to_sec() ) )
                
                if topic in save_topics:
                    outbag.write(topic, msg, t)


                if topic == '/imu/data_raw':
                    imu_msg = Imu()
                    imu_msg.header = msg.header
                    # 角速度
                    imu_msg.angular_velocity.x = msg.angular_velocity.x * math.pi / 180.0
                    imu_msg.angular_velocity.y = msg.angular_velocity.y * math.pi / 180.0
                    imu_msg.angular_velocity.z = 1.0 * msg.angular_velocity.z * math.pi / 180.0
                    # 线加速度
                    imu_msg.linear_acceleration.x = msg.linear_acceleration.x
                    imu_msg.linear_acceleration.y = msg.linear_acceleration.y
                    imu_msg.linear_acceleration.z = msg.linear_acceleration.z 
                    outbag.write("/imu", imu_msg, t)


if __name__ == '__main__':
    # ... existing code ...
    
    # 定义输入和输出bag文件路径
    input_bag_file = "/mnt/nvme0n1p2/data/0917/2025-09-17-11-14-45.bag"
    output_bag_file = "/mnt/nvme0n1p2/data/0917/0917_park.bag"
    
    modify_and_save_bag(input_bag_file, output_bag_file)
    
    print("Modified bag file saved as:", output_bag_file)