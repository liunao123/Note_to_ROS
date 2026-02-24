# Labels功能说明

## 概述

Labels功能用于管理和导出数据采集过程中的标注数据。Labels文件以JSON格式存储，命名方式与odoms文件相同：`frame_id_timestamp.json`

## 数据结构

### 目录结构
每个采集会话的数据目录中需要包含`labels`文件夹：

```
session_dir/
├── odoms/
│   ├── 0_1763951779.000.yaml
│   ├── 1_1763951779.100.yaml
│   └── ...
├── labels/                    # 新增的labels目录
│   ├── 0_1763951779.000.json  # 与odoms对应的标注文件
│   ├── 1_1763951779.100.json
│   └── ...
├── images/
├── pointclouds/
├── sparse/
└── calib/
```

### 文件命名规则
- 格式：`{frame_id}_{timestamp}.json`
- 示例：`0_1763951779.000.json`
- 必须与对应的odom文件名（除扩展名外）完全一致

### JSON内容
Labels文件的JSON内容可以根据实际需求定制，例如：

```json
{
  "frame_id": 0,
  "timestamp": 1763951779.000,
  "objects": [
    {
      "id": 1,
      "type": "car",
      "bbox": [100, 200, 300, 400],
      "confidence": 0.95
    }
  ],
  "annotations": {
    "weather": "sunny",
    "time_of_day": "afternoon"
  }
}
```

## 数据库Schema

### Labels表结构

```sql
CREATE TABLE labels (
    label_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    pose_id BIGINT REFERENCES vehicle_poses(pose_id) ON DELETE CASCADE,
    
    frame_id INTEGER NOT NULL,
    timestamp_sec BIGINT NOT NULL,
    timestamp_nsec INTEGER DEFAULT 0,
    
    label_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,
    absolute_path TEXT NOT NULL,
    
    file_size_bytes BIGINT,
    label_data JSONB,  -- 可选：存储JSON内容
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_frame_label UNIQUE (session_id, frame_id)
);
```

## 使用方法

### 1. 更新现有数据库

如果你已经有一个运行中的数据库，需要先添加labels表：

```bash
psql -d gtxc_db -U tyjt -f add_labels_table.sql
```

### 2. 导入数据

使用 `01_data_importer.py` 导入包含labels的会话数据：

```bash
python 01_data_importer.py
```

导入器会自动：
- 检测 `session_dir/labels/` 目录
- 解析所有 `*.json` 文件
- 将文件信息和（可选）JSON内容存入数据库
- 如果labels目录不存在，会跳过不报错

### 3. 导出ROI数据

使用 `03_get_roi_data.py` 导出位姿时，会自动包含对应的labels文件：

```bash
python 03_get_roi_data.py --lat 31.41 --lon 120.66 --distance 100
```

导出的目录结构：

```
export/search_31.410333_120.655821_100m/
├── odoms/
│   ├── 0_1763951779.000.yaml
│   └── ...
├── labels/                    # 自动导出的labels
│   ├── 0_1763951779.000.json
│   └── ...
├── images/
├── pointclouds/
├── sparse/
├── calib/
└── export_report.json
```

### 4. 查看导出统计

导出报告 `export_report.json` 会包含labels的统计信息：

```json
{
  "export_info": {
    "output_directory": "/path/to/export",
    "total_poses": 100,
    "total_size_bytes": 1234567890,
    "total_size_human": "1.15 GB"
  },
  "file_counts": {
    "odoms": 100,
    "optimized_poses": 100,
    "pointclouds": 100,
    "images": 400,
    "labels": 100
  }
}
```

## 注意事项

1. **文件命名一致性**：Labels文件名必须与对应的odom文件名（除扩展名）完全一致
   
2. **可选存储JSON内容**：导入时可以选择将JSON内容存储到数据库的`label_data`字段（JSONB类型），方便查询。如果文件很大，建议只存储文件路径。

3. **向后兼容**：如果旧数据没有labels目录，导入和导出都会正常工作，不会报错

4. **批量处理**：导入器支持批量插入，默认batch_size=1000

5. **错误处理**：如果某个labels文件格式错误或无法读取，会记录警告但继续处理其他文件

## 数据库查询示例

### 查询某个会话的所有labels

```sql
SELECT 
    l.frame_id,
    l.timestamp_sec,
    l.label_name,
    l.absolute_path,
    l.file_size_bytes
FROM labels l
WHERE l.session_id = 1
ORDER BY l.frame_id;
```

### 查询包含特定标注内容的labels

如果存储了JSON内容到`label_data`字段：

```sql
SELECT 
    l.frame_id,
    l.label_data->'annotations'->>'weather' as weather
FROM labels l
WHERE l.session_id = 1
  AND l.label_data->'annotations'->>'weather' = 'sunny';
```

### 联合查询位姿和标注

```sql
SELECT 
    vp.frame_id,
    vp.longitude,
    vp.latitude,
    l.label_name,
    l.label_data
FROM vehicle_poses vp
LEFT JOIN labels l ON vp.session_id = l.session_id AND vp.frame_id = l.frame_id
WHERE vp.session_id = 1
ORDER BY vp.frame_id;
```

## 开发说明

### 相关文件

1. **数据库Schema**: `00_mapping_system_schema.sql` - 包含labels表定义
2. **导入脚本**: `01_data_importer.py` - 包含`import_labels()`方法
3. **导出脚本**: `03_get_roi_data.py` - 在`_export_single_pose()`中处理labels
4. **更新脚本**: `add_labels_table.sql` - 用于更新现有数据库

### 扩展建议

1. 可以在`label_data`中使用JSONB的索引功能加速查询
2. 可以添加labels的验证逻辑，确保JSON格式符合要求
3. 可以添加labels的统计视图，类似于`session_statistics`

## 更新日志

- 2024-12-31: 初始版本，添加labels表支持
  - 数据库schema更新
  - 导入功能实现
  - 导出功能实现
  - 文档编写
