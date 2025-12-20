
# 简单使用说明：
# 只需要指定位置和搜索的范围即可，输出结果会保存在config.yaml指定的输出目录中。

# 在config.yaml中配置与03_get_roi_data.py相关的参数
# 就下面这几个参数
# ===================================================================
# 位姿搜索与复制配置 (03_get_roi_data.py)
# ===================================================================
# search_params:
#   # 默认搜索位置（苏州高铁新城某个点）
#   default_location:
#     latitude: 31.41033324
#     longitude: 120.65582103
#   distance: 100.0  # 默认搜索半径（米）
#   # UTM区号会根据经度自动计算，无需手动配置
#   limit: null      # 结果数量限制，null 表示不限制

# 依次运行即可

# /usr/bin/python3 /data/postgresql/03_get_roi_data.py 

/usr/bin/python3 /data/postgresql/04_merge_session_data.py

# /usr/bin/python3 /data/postgresql/06_export_poses_to_kml.py

# /usr/bin/python3 /data/postgresql/07_get_surround_hd_map.py 
