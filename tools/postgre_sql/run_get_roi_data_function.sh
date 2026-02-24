
# 简单使用说明：
# 只需要指定位置和搜索的范围即可，输出结果会保存在 config/config.yaml 指定的输出目录中。

# 依次运行即可
# /usr/bin/python3 ./00_test_connection.py
# /usr/bin/python3 ./02_show_status.py

# 新数据导入
# /usr/bin/python3 ./01_data_importer.py

# 根据位置获取感兴趣区域数据，包括高精地图数据
/usr/bin/python3 ./03_get_roi_data.py

# #  重命名点云文件和图像文件，按时间排序编号（已统一风格）
# 结果输出到 /data/3dgs_data//search_xxx   
/usr/bin/python3 ./04_resorted_by_time.py

# # 获取数据给 3DGS 生成 img_whitelist_for_3dgs.txt
# # 内部会 单独起一个线程 去调用 06_nu_sampling 进行非均匀采样
#  读取 /data/3dgs_data/search_xxx 目录下的数据
/usr/bin/python3 ./05_get_roi_data_for_3dgs.py

# 合并多个session的数据，生成las pcd ply等文件
# /usr/bin/python3 ./07_merge_session_data.py

# # 导出相机位姿到KML文件 可以在google earth中查看数据的采集路线
# /usr/bin/python3 ./10_export_poses_to_kml.py

# 测试使用
# /usr/bin/python3 ./08_grid_data_100m.py

# sync -avhP   13/ 14/   tyjt@172.26.9.101:/media/tyjt/TOSHIBA_LN/test_id4/
