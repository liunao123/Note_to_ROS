#!/bin/bash

# 记录文件所在目录
RECORD_DIR="/home/tyjt/Desktop/drivewise_v101/data/W7_2_master/"
# 输出rosbag目录
OUTPUT_DIR="./rosbags"
# 合并后的bag文件名
MERGED_BAG="/mnt/nvme0n1p2/W7_2_m.bag"

mkdir -p "$OUTPUT_DIR"

# 转换所有文件（不区分扩展名）
for record_file in "$RECORD_DIR"/*; do
    if [[ -f "$record_file" ]]; then
        base_name=$(basename "$record_file")
        echo $record_file
        echo $OUTPUT_DIR/$base_name.bag
        bag_convert -m=r2b -r="$record_file" -b="$OUTPUT_DIR/$base_name.bag"
    fi
done

# 按文件名排序合并所有bag
python  merge_bags.py  -o $MERGED_BAG  -i ./$OUTPUT_DIR  -c none

echo "转换和合并完成，合并包文件: $MERGED_BAG"
