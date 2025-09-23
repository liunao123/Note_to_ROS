#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import argparse

from cyber_record.record import Reader
from cyber_record.record import Record


# 指定输入文件夹
# 指定输出文件
INPUT_record_path = './apollo_record'
INPUT_record_name = 'merged_record'


def get_files_list(arg_input):
    # 只读取指定文件夹下的所有文件（不区分扩展名），按文件名排序
    if not os.path.isdir(arg_input):
        raise ValueError(f"{arg_input} 不是一个有效的文件夹路径")
    files = [os.path.join(arg_input, f) for f in os.listdir(arg_input) if os.path.isfile(os.path.join(arg_input, f))]
    files.sort()  # 按文件名排序
 
    # print("bags list:")
    # for i, f in enumerate(files):
    #     print(f"{i} . {f}")
    return files

# 创建写入器
with Record(INPUT_record_name, mode='w') as writer:
    # 遍历每个输入文件
    record_files =  get_files_list(INPUT_record_path)

    for file in record_files:
        with Record(file) as reader:
            print("reading:", file)
            for topic, message, t in reader.read_messages():
                    writer.write(topic, message, t)
