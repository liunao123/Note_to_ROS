#!/usr/bin/python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np

# 示例数据
categories = ['A', 'B', 'C', 'D', 'E']  # 类别
values = [25, 40, 30, 35, 20]  # 每个类别的值
errors = [2, 3, 1, 2, 1]  # 每个类别的误差范围

# 绘制柱状图
plt.bar(categories, values, yerr=errors, capsize=15)

# 设置坐标轴标签
plt.xlabel('Categories')
plt.ylabel('Values')

# 设置标题
plt.title('Bar Plot with Error Bars')

# 显示图形
plt.show()

#https://ntu-aris.github.io/ntu_viral_dataset/
