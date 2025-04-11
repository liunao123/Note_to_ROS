#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt

file_path = []
# 添加字符串到列表中
file_path.append("memory_usage_faster_lio.log")
file_path.append("memory_usage_fast_lio.log")
file_path.append("memory_usage_ig_lio.log")
file_path.append("memory_usage_point_lio.log")
print(file_path)

# 读取4个文件
for i in range(0, 4):
    # file_path = f"file_{i}.txt"
    data = []
    with open(file_path[i], 'r') as file:
        # 读取第三列数据
        column_three = [float(line.split()[3]) / 1000.0 for line in file]
        data.extend(column_three  )

    # 画出每个文件对应的线
    plt.plot(data, label=file_path[i])

# 添加图例
plt.legend(fontsize=16)
plt.grid()

plt.xlabel('Time(s)', fontsize=14)
plt.ylabel('Memory(MB)', fontsize=14)
plt.title('Memory Used for 800s data', fontsize=20)
ax = plt.gca()
ax.tick_params(axis='x', labelsize=12)  # 更改x轴标签的大小为12
ax.tick_params(axis='y', labelsize=12)  # 更改y轴标签的大小为12
plt.show()