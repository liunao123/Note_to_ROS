#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

def read_tum_trajectory(file_path):
    timestamps = []
    positions = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith("#"):
                continue
            data = line.strip().split()
            timestamps.append(float(data[0]))
            positions.append([float(data[1]), float(data[2]), float(data[3])])

    return timestamps, positions

def calculate_errors(reference_positions, estimated_positions):
    errors = []
    for ref_pos, est_pos in zip(reference_positions, estimated_positions):
        error = np.linalg.norm(np.array(ref_pos) - np.array(est_pos))
        errors.append(error)
    return errors

def plot_error_with_colorbar(timestamps, reference_positions, estimated_positions):
    errors = calculate_errors(reference_positions, estimated_positions)

    cmap = plt.cm.get_cmap('viridis')
    normalize = Normalize(vmin=min(errors), vmax=max(errors))
    colors = [cmap(normalize(error)) for error in errors]

    fig, ax = plt.subplots()
    sc = ax.scatter(timestamps, errors, c=errors, cmap=cmap, norm=normalize)
    cbar = plt.colorbar(ScalarMappable(cmap=cmap, norm=normalize), ax=ax)
    cbar.set_label('Error')

    for i in range(len(timestamps)):
        ax.plot([timestamps[i], timestamps[i]], [0, errors[i]], color=colors[i])

    ax.set_xlabel('Time')
    ax.set_ylabel('Error')
    ax.set_title('Error between Reference and Estimated Trajectories')
    plt.show()

# 读取参考轨迹和估计轨迹数据
est_timestamps, est_positions = read_tum_trajectory('/opt/csg/slam/navs/livos/result/sbs_01/faster_lio.txt')
ref_timestamps, ref_positions = read_tum_trajectory('/opt/csg/slam/navs/livos/result/sbs_01/ground_truth.txt')

# 确保时间戳对齐
# assert ref_timestamps == est_timestamps, "Timestamps of reference and estimated trajectories do not match!"

# 绘制误差图与颜色条
plot_error_with_colorbar(ref_timestamps, ref_positions, est_positions)