#!/bin/bash

while true
do
    # 获取当前时间戳
    # timestamp=$(date +"%Y-%m-%d %H:%M:%S")
    timestamp=$(date +%s)


    # 获取程序的内存占用情况
    memory_usage=$(ps aux | grep run_mapping_online | grep -v grep | awk '{print $6}')

    # 将时间戳和内存占用写入日志文件
    echo "$timestamp 0 0 $memory_usage 0 0 0 1" >> memory_usage.log

    sleep 1  # 每5秒记录一次
done