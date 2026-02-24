# GPS坐标数组 (格式: "纬度 经度")
GPS_COORDS=(
 "Saved Grid [0,0]: 44 points, 31.425274 120.619095 50 70" 
 "Saved Grid [0,1]: 124 points, 31.426251 120.619070 57 77" 
 "Saved Grid [1,0]: 145 points, 31.426415 120.620117 50 70" 
 "Saved Grid [1,1]: 158 points, 31.427327 120.620094 50 70" 
 "Saved Grid [1,2]: 176 points, 31.428243 120.620071 51 71" 
 "Saved Grid [2,0]: 128 points, 31.425891 120.621182 50 70" 
 "Saved Grid [2,1]: 225 points, 31.428997 120.621104 50 70" 
 "Saved Grid [3,0]: 126 points, 31.425629 120.622241 50 70" 
 "Saved Grid [3,1]: 158 points, 31.429085 120.622154 50 70" 
 "Saved Grid [4,0]: 126 points, 31.425379 120.623299 50 70" 
 "Saved Grid [4,1]: 156 points, 31.428722 120.623215 50 70" 
 "Saved Grid [5,0]: 137 points, 31.425120 120.624357 50 70" 
 "Saved Grid [5,1]: 147 points, 31.428383 120.624275 50 70" 
 "Saved Grid [6,0]: 173 points, 31.424854 120.625416 50 70" 
 "Saved Grid [6,1]: 145 points, 31.428075 120.625335 50 70" 
 "Saved Grid [7,0]: 474 points, 31.424902 120.626466 50 70" 
 "Saved Grid [7,1]: 395 points, 31.425801 120.626444 50 70" 
 "Saved Grid [7,2]: 391 points, 31.426709 120.626421 50 70" 
 "Saved Grid [7,3]: 364 points, 31.427538 120.626400 50 70" 
 "Saved Grid [8,0]: 180 points, 31.424299 120.627533 50 70" 
 "Saved Grid [8,1]: 179 points, 31.427396 120.627455 50 70" 
 "Saved Grid [9,0]: 226 points, 31.424037 120.628592 50 70" 
 "Saved Grid [9,1]: 162 points, 31.427029 120.628516 50 70" 
 "Saved Grid [10,0]: 231 points, 31.423825 120.629649 50 70" 
 "Saved Grid [10,1]: 320 points, 31.424728 120.629626 50 70" 
 "Saved Grid [10,2]: 317 points, 31.425631 120.629603 50 70" 
 "Saved Grid [10,3]: 284 points, 31.426474 120.629582 50 70" 
 "Saved Grid [11,0]: 141 points, 31.423275 120.630714 50 70" 
 "Saved Grid [11,1]: 200 points, 31.426175 120.630641 50 70" 
 "Saved Grid [12,0]: 149 points, 31.423555 120.631805 54 74" 
 "Saved Grid [12,1]: 162 points, 31.424458 120.631783 54 74" 
 "Saved Grid [12,2]: 249 points, 31.425368 120.631760 54 74" 
 "Saved Grid [12,3]: 49 points, 31.425916 120.631746 54 74" 
)

# 设置最大处理数量 (0或负数表示处理全部)
MAX_COUNT=-1

# 循环处理每个GPS坐标
count=0
for coord in "${GPS_COORDS[@]}"; do
    # 提取最后4个字段：纬度 经度 roi_distance_min roi_distance_max
    lat=$(echo $coord | awk '{print $(NF-3)}')
    lon=$(echo $coord | awk '{print $(NF-2)}')
    roi_min=$(echo $coord | awk '{print $(NF-1)}')
    roi_max=$(echo $coord | awk '{print $NF}')
    
    count=$((count + 1))
    echo "[$count/${#GPS_COORDS[@]}] Processing: lat=$lat, lon=$lon, roi_min=$roi_min, roi_max=$roi_max"
    
    /usr/bin/python3 ./03_get_roi_data.py --lat $lat --lon $lon --roi_distance_min $roi_min --roi_distance_max $roi_max
    /usr/bin/python3 ./04_resorted_by_time.py --lat $lat --lon $lon --roi_distance_min $roi_min --roi_distance_max $roi_max
    /usr/bin/python3 ./05_get_roi_data_for_3dgs.py --lat $lat --lon $lon --roi_distance_min $roi_min --roi_distance_max $roi_max
    
    # 检查是否达到最大处理数量
    if [ $MAX_COUNT -gt 0 ] && [ $count -ge $MAX_COUNT ]; then
        echo "已达到最大处理数量 $MAX_COUNT，停止处理"
        break
    fi
    sleep 10  # 可选：每次处理后暂停10秒，避免过快连续请求
done



# 简单使用说明：
# 只需要指定位置和搜索的范围即可，输出结果会保存在 config/config.yaml 指定的输出目录中。

# 依次运行即可
# /usr/bin/python3 ./00_test_connection.py
# /usr/bin/python3 ./02_show_status.py

# 新数据导入
# /usr/bin/python3 ./01_data_importer.py

# 根据位置获取感兴趣区域数据，包括高精地图数据
# /usr/bin/python3 ./03_get_roi_data.py --lon 120.5 --lat 31.5 --roi_distance_min 150 --roi_distance_max 250

# # #  重命名点云文件和图像文件，按时间排序编号（已统一风格）
# # 结果输出到 /data/3dgs_data//search_xxx   
# /usr/bin/python3 ./04_resorted_by_time.py --lon 120.5 --lat 31.5 --roi_distance_min 150 --roi_distance_max 250

# # # 获取数据给 3DGS 生成 img_whitelist_for_3dgs.txt
# # # 内部会 单独起一个线程 去调用 06_nu_sampling 进行非均匀采样
# #  读取 /data/3dgs_data/search_xxx 目录下的数据

# /usr/bin/python3 05_get_roi_data_for_3dgs.py --lon 120.5 --lat 31.5 --roi_distance_min 150 --roi_distance_max 250
 
