#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Time
import rosbag
import math
from chcnav.msg import hcinspvatzcb # 这就是你的自定义消息
from sensor_msgs.msg import Imu
import os

# heading 转换为 yaw
# by ok 20250917
def convertHeadingToYaw( heading ):
    yaw = -1.0 * heading
    if yaw > 180.0:
        yaw = yaw - 180.0
    elif yaw < -180.0:
        yaw = yaw + 360.0
    return yaw
 

save_topics = {"/bynav/inspvax", "/lidar_points", "/gps/fix", "/gps/gps", "/bynav/heading2", "/odom", "/lidar_imu" , "/bynav/bestpos","/bynav/inspva" }  
# save_topics = {"/lidar_points", "/odom"}  

if __name__ == '__main__':
    # source ~/Desktop/ros_ws/devel/setup.bash
    # 使用前请source chcnav ros环境

    input_dir = "/media/tyjt/Elements1/nongan/1028/1/"
    merged_bag_file = "/media/tyjt/Elements/nongan_rewrite_bag/1028_1.bag"
    print("\ninput_dir %s " % (input_dir))
    print("merged_bag_file %s \n\n" % (merged_bag_file))

    with rosbag.Bag(merged_bag_file, 'w') as outbag:
        bag_files = [f for f in os.listdir(input_dir) if f.endswith('.bag')]
        bag_files.sort()
        cnt = 1
        for filename in bag_files:
            input_bag_file = os.path.join(input_dir, filename)
            print("Processing %d/%d : %s" % (cnt, len(bag_files), input_bag_file))
            cnt = cnt + 1
            # continue
            # 复用modify_and_save_bag的核心逻辑，但写入同一个outbag
            with rosbag.Bag(input_bag_file, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    if topic == '"/chcnav/devpvt"':
                        continue
                    
                    if topic == '/bynav/inspvax':
                        hc_msg = hcinspvatzcb()
                        hc_msg.header = msg.header
                        hc_msg.week = msg.nov_header.gps_week_number
                        hc_msg.second = msg.nov_header.gps_week_milliseconds / 1000.0
                        hc_msg.longitude = msg.longitude
                        hc_msg.latitude = msg.latitude
                        hc_msg.altitude = msg.height + msg.undulation
                        hc_msg.roll = msg.roll
                        hc_msg.pitch = msg.pitch
                        hc_msg.yaw = convertHeadingToYaw(msg.azimuth)
                        hc_msg.speed = math.sqrt(msg.north_velocity * msg.north_velocity + msg.east_velocity * msg.east_velocity + msg.up_velocity * msg.up_velocity)

                        hc_msg.position_stdev = [0.0, 0.0, 0.0]  # Initialize as list first
                        hc_msg.position_stdev[0] = msg.latitude_stdev
                        hc_msg.position_stdev[1] = msg.longitude_stdev
                        hc_msg.position_stdev[2] = msg.height_stdev
                        
                        # Initialize stat array properly
                        hc_msg.stat = [0, 0]  # Initialize as list first
                        hc_msg.stat[0] = msg.ins_status.status
                        hc_msg.stat[1] = msg.pos_type.type

                        outbag.write("/chcnav/devpvt", hc_msg, Time.from_sec(t.to_sec()))

                    if topic in save_topics:
                        outbag.write(topic, msg, t)
                    
                    if topic == '/imu/data_raw':
                        imu_msg = Imu()
                        imu_msg.header = msg.header
                        imu_msg.angular_velocity.x = msg.angular_velocity.y * (  2.88991928100586 /  3.0517578125) * math.pi / 180.0 
                        imu_msg.angular_velocity.y = -1.0 * msg.angular_velocity.x * (  2.88991928100586/  3.0517578125 ) * math.pi / 180.0
                        imu_msg.angular_velocity.z = 1.0 * msg.angular_velocity.z * (  2.88991928100586 /  3.0517578125 ) * math.pi / 180.0
                        imu_msg.linear_acceleration.x = msg.linear_acceleration.y * ( 4.67617511749267 / 3.74094009399414)
                        imu_msg.linear_acceleration.y = -1.0 *  msg.linear_acceleration.x * ( 4.67617511749267 / 3.74094009399414 )
                        imu_msg.linear_acceleration.z = msg.linear_acceleration.z * ( 4.67617511749267 / 3.74094009399414 )
                        outbag.write("/gnss_imu", imu_msg, t)
    print("All bags merged and saved as:", merged_bag_file)
