# PostGIS 移动测绘数据管理系统

基于 PostgreSQL/PostGIS 的移动测绘数据管理和查询系统。

## 配置文件

**`config.yaml`** - 系统统一配置文件，所有脚本从此文件读取参数

## 脚本使用说明

### 00_test_connection.py - 测试数据库连接
```bash
/usr/bin/python3 00_test_connection.py
```
验证数据库配置是否正确，检查 PostgreSQL 和 PostGIS 状态。

---

### 01_data_importer.py - 批量导入数据，新的session放进去，也运行一下这个
```bash
/usr/bin/python3 01_data_importer.py
```
从文件系统导入移动测绘数据：
- 传感器配置和标定数据
- 车辆位姿（odom 和优化位姿）
- 图像和点云索引
- 子地图数据

**配置项**: `data_import.data_root`, `data_import.project_name`

---

### 02_show_status.py - 查看数据状态
```bash
/usr/bin/python3 02_show_status.py
```
显示数据库统计信息：项目、会话、位姿、图像、点云数量等。

---

### 03_get_roi_data.py - 搜索并导出数据
```bash
# 使用默认配置
/usr/bin/python3 03_get_roi_data.py

# 自定义参数
/usr/bin/python3 03_get_roi_data.py --lat 31.42 --lon 120.66 --distance 50 -o /path/to/output
```
基于 GPS 坐标搜索位姿并导出相关文件（图像、点云、odom）。

**配置项**: `search_params`, `data_export`

---

### 04_merge_session_data.py - 合并点云
```bash
# 使用默认配置
/usr/bin/python3 04_merge_session_data.py

# 自定义参数
/usr/bin/python3 04_merge_session_data.py -i /input/dir -o /output/dir -r 10
```
将多个点云文件合并为完整地图，输出 PCD 和 LAS 格式。

**配置项**: `merge_session`

---

### 05_manage_source_paths.py - 管理源路径
```bash
# 列出所有会话
/usr/bin/python3 05_manage_source_paths.py --list

# 更新源路径
/usr/bin/python3 05_manage_source_paths.py --update-all
```
管理数据的原始存储路径信息。

**配置项**: `source_path_management`

---

## 快速开始

1. **配置系统**: 编辑 `config.yaml`
2. **测试连接**: `/usr/bin/python3 00_test_connection.py`
3. **导入数据**: `/usr/bin/python3 01_data_importer.py`
4. **查看状态**: `/usr/bin/python3 02_show_status.py`
5. **搜索导出**: `/usr/bin/python3 03_get_roi_data.py`
6. **合并点云**: `/usr/bin/python3 04_merge_session_data.py`

## 命令行帮助

所有脚本支持 `--help` 查看详细参数：
```bash
/usr/bin/python3 03_get_roi_data.py --help
```

## 删除会话

使用 SQL 命令删除指定会话及其所有关联数据：
```sql
-- 方法1：按名称删除
DELETE FROM data_sessions WHERE session_name = '会话名称';

-- 方法2：按路径删除
DELETE FROM data_sessions WHERE session_path LIKE '%目录名%';

-- 方法3：按ID删除
DELETE FROM data_sessions WHERE session_id = 5;
```

或使用命令行：
```bash
psql -d gtxc_db -U tyjt -c "DELETE FROM data_sessions WHERE session_name = '会话名称';"
```

⚠️ **删除操作不可恢复**，会同时删除该会话的所有位姿、图像、点云索引数据。

## 数据库结构

- `projects` - 项目
- `data_sessions` - 会话
- `sensors` - 传感器
- `vehicle_poses` - 车辆位姿
- `optimized_poses` - 优化位姿
- `images` - 图像索引
- `point_clouds` - 点云索引
- `submaps` - 子地图

详细说明: `00_mapping_system_schema.sql`

## 系统要求

- PostgreSQL 14+ with PostGIS 3.0+
- /usr/bin/python3 3.8+
- 依赖: psycopg2, pyyaml, numpy, laspy, pyproj

---

**项目**: 苏州高铁新城移动测绘  
**更新**: 2025-12-09
