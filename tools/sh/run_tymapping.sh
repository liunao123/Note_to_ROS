#!/bin/bash

run_tymapping() {
    local TARGET_DIR="$1"
    local SOURCE_FOLDER="$2"

    # 检查目录存在性
    if [ ! -d "$TARGET_DIR" ] || [ ! -d "$SOURCE_FOLDER" ]; then
        echo "错误: 目录不存在"
        return 1
    fi

    local FOLDER_NAME
    FOLDER_NAME=$(basename "$SOURCE_FOLDER")
    local target_name
    target_name=$(basename "$TARGET_DIR")

    echo "processing --------: $TARGET_DIR"
    echo "copying calib file to : $TARGET_DIR/calib"

    # 复制文件夹
    cp -r "$SOURCE_FOLDER" "$TARGET_DIR/calib"

    # 查找所有db3文件，按文件名排序，并用逗号分隔
    db3_files=$(find "$TARGET_DIR" -maxdepth 1 -name "*.db3" -exec basename {} \; | sort | tr '\n' ',' | sed 's/,$//')
    if [ -z "$db3_files" ]; then
        db3_files="${target_name}.db3"
    fi
    echo "找到的db3文件: $db3_files"

    # 生成config.json
    cat > "$TARGET_DIR/config.json" << EOF
{
    "dataset_config": {
        "root_path":                 "$TARGET_DIR/",
        "calibration_file":          "/calib",
        "bag_file_name":             "$db3_files",
        "output_path":               "./",
        "sampling_interval_meters":  2.0,
        "sampling_interval_degrees": 5.0
    },
    "topic_config": {
        "point_cloud_topic": "/hesai/pandar_points",
        "odom_topic":        "/chcnav/devpvt",
        "camera_topics":     ["/cam1/pylon_camera","/cam2/pylon_camera","/cam3/pylon_camera","/cam4/pylon_camera","/cam5/pylon_camera","/cam6/pylon_camera","/cam7/pylon_camera"]
    }
}
EOF

    export RDP_INSTALL_DIR=/opt/raw-data-preprocessing-dev-id4/
    source /opt/ros/humble/setup.bash
    source $RDP_INSTALL_DIR/setup.bash

    echo " try mapping -------- "
    cd "$TARGET_DIR" || return 1
    pwd

    ros2 launch bag_parser launch.py config_file:="$TARGET_DIR/config.json"
    echo "start ty_mapping for $TARGET_DIR"
    ty_mapping -i "$TARGET_DIR"
    # Map_Completion 可选
    # Map_Completion -i "$TARGET_DIR" -r

    echo "DONE MAPPING FOR $TARGET_DIR"
    echo "完成!"
}

# 示例循环调用
# 你可以根据实际情况修改folders和calibs的内容
DB3_folders=(
    "/data/dwm_raw_data/1202_id4/park_ros2/"
)
calibs=(
    "./calib_id4/"
)

for ((i=0; i<${#folders[@]}; i++)); do
    run_tymapping "${DB3_folders[$i]}" "${calibs[0]}" 
done



