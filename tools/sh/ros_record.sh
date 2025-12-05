#!/bin/bash

# 按大小分割
rosbag record --split --size=1024 -a -o /path/to/output/folder

# 按时间分割
rosbag record --split --duration=30 -a -o /path/to/output/folder

# 带点压缩的录制
rosbag record  --split --duration=300 --lz4 -o bagname.bag  /cam4/pylon_camera /cam5/pylon_camera  /chcnav/devpvt
