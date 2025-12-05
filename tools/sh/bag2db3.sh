#!/bin/bash

# ROS1 bag 转 ROS2 bag 批量转换脚本（支持多目录批量处理）

convert_and_cleanup() {
    local SRC_DIR="$1"
    local DST_DIR="$2"

    # 删除目标文件夹
    rm -rf "$DST_DIR"

    # 检查源文件夹是否存在
    if [ ! -d "$SRC_DIR" ]; then
        echo "错误: 源文件夹 '$SRC_DIR' 不存在!"
        return 1
    fi

    # 创建目标文件夹
    mkdir -p "$DST_DIR"
    echo "目标文件夹: $DST_DIR"

    # 检查 rosbags-convert 命令是否存在
    if ! command -v rosbags-convert &> /dev/null; then
        echo "错误: rosbags-convert 命令未找到!"
        echo "请确保已安装 rosbags 工具："
        echo "pip install rosbags"
        return 1
    fi

    echo "开始转换..."

    for bag_file in "$SRC_DIR"/*.bag; do
        if [ ! -f "$bag_file" ]; then
            echo "警告: 在 '$SRC_DIR' 中未找到 .bag 文件"
            continue
        fi

        filename=$(basename "$bag_file" .bag)
        dst_file="$DST_DIR/${filename}"

        if [ -f "$dst_file" ]; then
            echo "  跳过，已存在"
            continue
        fi

        if rosbags-convert --src "$bag_file" --dst "$dst_file" >/dev/null 2>&1; then
            echo "  完成"
            echo "目标: $dst_file"
            echo "目标: $DST_DIR"
        else
            echo "  转换失败"
            [ -f "$dst_file" ] && rm -f "$dst_file"
        fi
    done

    echo "完成!"

    # 将 DST_DIR 目录下所有子目录中的 .db3 文件上移到 DST_DIR 目录
    find "$DST_DIR" -mindepth 2 -type f -name "*.db3" -exec mv {} "$DST_DIR"/ \;
    echo "所有子目录下的 .db3 文件已移动到: $DST_DIR"

    # 删除 DST_DIR 目录下的所有空子目录
    find "$DST_DIR" -mindepth 1 -type d -exec rm -rf {} +
    echo "所有子目录已删除，只保留 $DST_DIR 下的 .db3 文件"
    echo ""
    echo "----------------------------------------"
    echo ""

}

# 示例：批量处理多个目录
# 你可以根据需要修改下面的数组
SRC_DIR_LIST=(
    "/data/dwm_raw_data/1202_id4/park"
)
DST_DIR_LIST=(
    "/data/dwm_raw_data/1202_id4/park_ros2"
)

for i in "${!SRC_DIR_LIST[@]}"; do
    convert_and_cleanup "${SRC_DIR_LIST[$i]}" "${DST_DIR_LIST[$i]}"
done


