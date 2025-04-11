#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 读取txt文件
with open('/opt/csg/slam/navs/0000000005.txt', 'r') as file:
    lines = file.readlines()

# 写入pcd文件
with open('output.pcd', 'w') as pcd_file:
    # PCD文件头部
    pcd_file.write("# .PCD v0.7 - Point Cloud Data file format\n")
    pcd_file.write("VERSION 0.7\n")
    pcd_file.write("FIELDS x y z intensity\n")
    pcd_file.write("SIZE 4 4 4 4\n")
    pcd_file.write("TYPE F F F F\n")
    pcd_file.write("COUNT 1 1 1 1\n")
    pcd_file.write("WIDTH " + str(len(lines)) + "\n")
    pcd_file.write("HEIGHT 1\n")
    pcd_file.write("POINTS " + str(len(lines)) + "\n")
    pcd_file.write("DATA ascii\n")

    # 写入每个点的数据
    for line in lines:
        x, y, z, intensity = line.split()
        # pcd_file.write(f"{x} {y} {z} {intensity}\n")
        intensity  = float(intensity) * 100
        pcd_file.write("{} {} {} {}\n".format(x, y, z, intensity ))
