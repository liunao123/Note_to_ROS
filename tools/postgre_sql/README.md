
# Geo-Database-Engine 移动测绘数据管理系统

基于 PostgreSQL/PostGIS 的移动测绘数据管理、ROI 检索导出、点云合并与轨迹导出工具集。

## 目录结构（2026-01-27）

```
geo-database-engine/
├── 00_test_connection.py          # 测试数据库连接/版本
├── 01_data_importer.py            # 导入会话数据到数据库（含 labels 可选）
├── 02_show_status.py              # 交互式查看数据库状态/计数/labels
├── 03_get_roi_data.py             # ROI 检索并导出数据（图像/点云/odom/labels/hd_map）
├── 04_resorted_by_time.py         # 按时间重排序并重命名数据
├── 05_get_roi_data_for_3dgs.py    # 为 3DGS 生成白名单/筛选
├── 07_merge_session_data.py       # 合并点云输出 pcd/ply/las
├── 08_grid_data_100m.py           # 100m 网格化数据处理
├── 10_export_poses_to_kml.py      # 导出轨迹到 KML（Google Earth）
├── run_get_roi_data_function.sh   # 一键执行常用流程（示例脚本）
├── config/
│   └── config.yaml                # 统一配置文件（所有脚本默认读取这里）
├── sql/
│   ├── 00_mapping_system_schema.sql
│   ├── add_labels_table.sql
│   ├── add_masks_table.sql
│   └── query_submaps.sql
├── tools/
│   ├── 00_prepare_db.sh
│   ├── install_sql.sh
│   └── manage_source_paths.py
├── qt/                            # 公共工具（配置加载/投影/传感器参数等）
├── doc/                           # 文档（labels说明、数据说明等）
├── data/                          # 示例/资源/输出
├── include/ src/ build/           # C++ 相关目录（如有）
└── CMakeLists.txt                 # C++ 工程构建（如有）
```

> 说明：所有 Python 脚本默认读取 `config/config.yaml`，如需自定义请修改 `qt/load_config.py`。


## 配置文件说明

- `config/config.yaml`：系统统一配置文件，所有脚本默认从此读取参数。
- 如需指定其他配置文件路径，可修改 `qt/load_config.py` 中 `load_config()` 的默认路径，或在脚本中显式传入。


## 常用流程（推荐）

1. **准备数据库/安装 Schema**
	- 创建数据库与启用 PostGIS 后，执行 `sql/00_mapping_system_schema.sql`
	- 如需 labels 功能：执行 `sql/add_labels_table.sql`

2. **测试连接**
	```bash
	python3 00_test_connection.py
	```

3. **导入数据**
	```bash
	python3 01_data_importer.py
	```

4. **查看状态（交互式）**
	```bash
	python3 02_show_status.py
	```

5. **ROI 检索并导出数据**
	```bash
	python3 03_get_roi_data.py
	```

6. **按时间重排序并重命名数据**
	```bash
	python3 04_resorted_by_time.py
	```

7. **3DGS 白名单筛选**
	```bash
	python3 05_get_roi_data_for_3dgs.py
	```

8. **合并点云/导出地图**
	```bash
	python3 07_merge_session_data.py
	```

9. **导出轨迹到 KML（Google Earth）**
	```bash
	python3 10_export_poses_to_kml.py
	```

## 脚本说明


### 00_test_connection.py
测试数据库连接，检查 PostgreSQL 和 PostGIS 状态。

### 01_data_importer.py
批量导入移动测绘数据，包括传感器配置、车辆位姿、图像、点云索引等。

### 02_show_status.py
交互式查看数据库统计信息：项目、会话、位姿、图像、点云数量等。

### 03_get_roi_data.py
基于 GPS 坐标搜索位姿并导出相关文件（图像、点云、odom、labels）。支持自定义参数。

### 04_resorted_by_time.py
按时间重排序并重命名所有相关数据文件，确保 ID 唯一且递增。

### 05_get_roi_data_for_3dgs.py
为 3DGS 生成白名单/筛选，输出可用于 3DGS 的图像路径列表。

### 07_merge_session_data.py
合并点云为完整地图，输出 PCD、PLY、LAS 格式。

### 9_export_poses_to_kml.py
导出轨迹为 KML 文件（Google Earth）。

### run_get_roi_data_function.sh
一键执行常用流程的示例脚本。


## 命令行帮助

所有脚本均支持 `--help` 查看详细参数：
```bash
python3 03_get_roi_data.py --help
```

## 删除会话

可通过 SQL 命令或 psql 工具删除指定会话及其所有关联数据：
```sql
DELETE FROM data_sessions WHERE session_name = '会话名称';
DELETE FROM data_sessions WHERE session_path LIKE '%目录名%';
DELETE FROM data_sessions WHERE session_id = 5;
```
```bash
psql -d gtxc_db -U tyjt -c "DELETE FROM data_sessions WHERE session_name = '会话名称';"
```
⚠️ **删除操作不可恢复**，会同时删除该会话的所有位姿、图像、点云索引数据。


## Labels（标注）功能

- Labels 表 schema 位于 `sql/add_labels_table.sql`
- Masks 表 schema 位于 `sql/add_masks_table.sql`
- 导入/导出细节说明见：`doc/README_LABELS_FEATURE.md`


## 数据库结构（核心表）

- `projects` - 项目
- `data_sessions` - 会话
- `sensors` - 传感器
- `vehicle_poses` - 车辆位姿
- `optimized_poses` - 优化位姿
- `images` - 图像索引
- `point_clouds` - 点云索引
- `submaps` - 子地图

详细说明见 `sql/00_mapping_system_schema.sql`


## 系统要求

- PostgreSQL 14+ with PostGIS 3.x
- Python 3.8+
- 依赖：psycopg2, pyyaml, numpy, pypcd4, laspy, pyproj, pcl, geopandas, shapely, PIL, OpenCV 等

---

**项目**: 苏州高铁新城移动测绘
**更新**: 2026-01-27
