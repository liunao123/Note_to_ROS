#!/bin/bash
# filepath: /home/tyjt/nongan_data/copy_sth.sh

# 复制到指定目标目录，排除指定后缀名的文件
# 用法: ./copy_sth.sh [排除的后缀名]
# 例如: ./copy_sth.sh .db3  (复制除了.db3以外的所有文件)

destination="/mnt/map_gtxc_/gtxc_20251120/Key_Frame"
TARGET_DIR="/home/tyjt/nongan_data/ros2_gtxc_1124/"

# 获取要排除的后缀名参数，默认排除.db3文件
EXCLUDE_SUFFIX="${1:-.db3}"

# 确保后缀名以点开头
if [[ "$EXCLUDE_SUFFIX" != .* ]]; then
    EXCLUDE_SUFFIX=".$EXCLUDE_SUFFIX"
fi

echo "源目录: $TARGET_DIR"
echo "目标目录: $destination"
echo "排除文件后缀: $EXCLUDE_SUFFIX"
echo "========================================="

mkdir -p "$destination"

for folder in "$TARGET_DIR"/*/; do
    if [ -d "$folder" ]; then
        folder_name=$(basename "$folder")
        echo "处理文件夹: $folder_name"

        # 创建目标文件夹
        mkdir -p "$destination/$folder_name"

        # 复制所有文件和文件夹，但排除指定后缀的文件
        for item in "$folder"/*; do
            if [ -e "$item" ]; then
                item_name=$(basename "$item")
                
                # 如果是文件且后缀匹配排除列表，跳过
                if [ -f "$item" ] && [[ "$item_name" == *"$EXCLUDE_SUFFIX" ]]; then
                    echo "  跳过文件: $item_name (排除 $EXCLUDE_SUFFIX 文件)"
                    continue
                fi
                
                # 检查目标是否已存在
                if [ -e "$destination/$folder_name/$item_name" ]; then
                    echo "  $item_name 已存在，跳过"
                else
                    if [ -d "$item" ]; then
                        echo "  复制文件夹: $item_name"
                        cp -r "$item" "$destination/$folder_name/"
                    else
                        echo "  复制文件: $item_name"
                        cp "$item" "$destination/$folder_name/"
                    fi
                fi
            fi
        done
        echo ""
    fi
done

echo "复制完成！"