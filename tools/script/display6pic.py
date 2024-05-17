#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import os


def display(folder_path):
    # 文件夹路径
    # folder_path = "./pic_test/"
    
    # 获取文件夹中的图像文件名
    image_files = [f for f in os.listdir(folder_path) if f.endswith('.jpg') or f.endswith('.png')]
    
    # 创建一个2x3的图像子图布局
    sub_row_cnts = 2
    sub_col_cnts = 3
    # fig, axs = plt.subplots(sub_row_cnts, sub_col_cnts)
    fig, axs = plt.subplots(sub_row_cnts, sub_col_cnts, figsize=(10,6))
    # fig = plt.figure()

    # manager = plt.get_current_fig_manager()
    # manager.full_screen_toggle()   
    
    
    # 循环读取和显示图像
    for i, image_file in enumerate(image_files):
        if i >= sub_row_cnts * sub_col_cnts:
            break
    
        image_path = os.path.join(folder_path, image_file)
        image = plt.imread(image_path)
        print(image_path)
    
        # 获取当前子图的行和列索引
        row = i // sub_col_cnts
        col = i % sub_col_cnts 
    
        # 在相应的子图上显示图像
        axs[row, col].imshow(image)
        axs[row, col].axis('off')
        # axs[row, col].set_title(image_file)
        axs[row, col].set_title(image_file[:2] + "-" + folder_path[-2:])
    
    # 调整子图之间的间距
    plt.subplots_adjust(wspace=0.05, hspace=0.1)
    
    # 获取显示器的分辨率
    # dpi = plt.gcf().dpi
    
    # # 保存为全分辨率的PNG格式
    # plt.savefig("全屏图像.png", dpi=dpi)
    
    # 显示图像
    plt.show()
    # time.sleep(5)
    # plt.close()


# 主文件夹路径
folder_path = "/opt/csg/slam/navs/zz_left/"

# 遍历文件夹及其所有子文件夹
sub_folders = []
for root, dirs, files in os.walk(folder_path):
    for dir in dirs:
        sub_folder_path = os.path.join(root, dir)
        sub_folders.append(sub_folder_path)

# 打印所有子文件夹
for sub_folder in sub_folders:
    print(sub_folder)
    display(sub_folder)



