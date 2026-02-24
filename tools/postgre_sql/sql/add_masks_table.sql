-- 添加masks表的SQL脚本
-- 用于更新现有数据库，添加masks支持
-- 使用方法: psql -d gtxc_db -U tyjt -f add_masks_table.sql

-- ============================================
-- 掩码数据 (Masks)
-- ============================================

-- 掩码文件表（存储masks文件夹下的掩码图像数据）
CREATE TABLE IF NOT EXISTS masks (
    mask_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    pose_id BIGINT REFERENCES vehicle_poses(pose_id) ON DELETE CASCADE,
    
    frame_id INTEGER NOT NULL,
    timestamp_sec BIGINT NOT NULL,
    timestamp_nsec INTEGER DEFAULT 0,
    
    sensor_name VARCHAR(100) NOT NULL,  -- 传感器名称（对应masks下的子文件夹）
    mask_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,  -- 相对于 session_path 的路径
    absolute_path TEXT NOT NULL,  -- 完整的文件系统路径
    
    file_size_bytes BIGINT,
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_frame_sensor_mask UNIQUE (session_id, frame_id, sensor_name)
);

CREATE INDEX IF NOT EXISTS idx_masks_session ON masks(session_id);
CREATE INDEX IF NOT EXISTS idx_masks_pose ON masks(pose_id);
CREATE INDEX IF NOT EXISTS idx_masks_timestamp ON masks(timestamp_sec);
CREATE INDEX IF NOT EXISTS idx_masks_frame ON masks(frame_id);
CREATE INDEX IF NOT EXISTS idx_masks_sensor ON masks(sensor_name);

-- 打印完成信息
\echo 'Masks表创建成功！'
\echo '现在可以使用 01_data_importer.py 重新导入数据，或手动导入masks文件'
