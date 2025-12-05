#!/bin/bash

destination="jmsroot@192.168.2.78:/data-new/nongan/"


# 遍历文件夹内所有指定后缀名的文件
# 用法: ./scp.sh [文件夹路径] [文件后缀名]
# 例如: ./scp.sh /path/to/folder .txt
# 或者: ./scp.sh /path/to/folder txt

# 设置默认值
DEFAULT_DIR="/home/tyjt/nongan_data/ros2_nongan"
DEFAULT_SUFFIX=".db3"

# 获取参数
TARGET_DIR="${1:-$DEFAULT_DIR}"
FILE_SUFFIX="${2:-$DEFAULT_SUFFIX}"

# 检查目录是否存在
if [ ! -d "$TARGET_DIR" ]; then
    echo "错误: 目录 '$TARGET_DIR' 不存在"
    echo "用法: $0 [文件夹路径] [文件后缀名]"
    echo "例如: $0 /path/to/folder .txt"
    exit 1
fi

# 确保后缀名以点开头
if [[ "$FILE_SUFFIX" != .* ]]; then
    FILE_SUFFIX=".$FILE_SUFFIX"
fi

echo "遍历目录: $TARGET_DIR"
echo "文件后缀: $FILE_SUFFIX"
echo "========================================="

# 初始化计数器
file_count=0

# 递归函数遍历所有子文件夹
process_directory() {
    local dir="$1"
    local suffix="$2"
    
    # echo "处理目录: $dir"
    
    # 遍历当前目录中的所有文件
    for file in "$dir"/*"$suffix"; do
        if [ -f "$file" ]; then
            filename=$(basename "$file")
            absolute_path=$(realpath "$file")
            echo "  找到文件: $absolute_path"
            echo "  复制到: $destination"
            ((file_count++))
            
            # 使用rsync进行高效的远程复制（支持断点续传）
            if rsync -avz --partial --progress "$absolute_path" "$destination"; then
                echo "  ✓ 复制成功"
            else
                echo "  ✗ 复制失败"
            fi
            echo ""
        fi
    done
    
    # 递归处理所有子目录
    for subdir in "$dir"/*/; do
        if [ -d "$subdir" ]; then
            process_directory "$subdir" "$suffix"
        fi
    done
}

# 开始递归处理
process_directory "$TARGET_DIR" "$FILE_SUFFIX"

echo ""
echo "========================================="
echo "完成！共找到 $file_count 个 $FILE_SUFFIX 文件"