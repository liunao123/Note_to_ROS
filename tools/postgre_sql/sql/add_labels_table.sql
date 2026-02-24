-- 添加labels表的SQL脚本
-- 用于更新现有数据库，添加labels支持
-- 使用方法: psql -d gtxc_db -U tyjt -f add_labels_table.sql

-- ============================================
-- 标注数据 (Labels)
-- ============================================

-- 标注文件表（JSON格式的标注数据，命名格式与odoms一样：id_time.json）
CREATE TABLE IF NOT EXISTS labels (
    label_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    pose_id BIGINT REFERENCES vehicle_poses(pose_id) ON DELETE CASCADE,
    
    frame_id INTEGER NOT NULL,
    timestamp_sec BIGINT NOT NULL,
    timestamp_nsec INTEGER DEFAULT 0,
    
    label_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,  -- 相对于 session_path 的路径
    absolute_path TEXT NOT NULL,  -- 完整的文件系统路径
    
    file_size_bytes BIGINT,
    label_data JSONB,  -- 标注JSON内容（可选存储）
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_frame_label UNIQUE (session_id, frame_id)
);

CREATE INDEX IF NOT EXISTS idx_labels_session ON labels(session_id);
CREATE INDEX IF NOT EXISTS idx_labels_pose ON labels(pose_id);
CREATE INDEX IF NOT EXISTS idx_labels_timestamp ON labels(timestamp_sec);
CREATE INDEX IF NOT EXISTS idx_labels_frame ON labels(frame_id);

-- 打印完成信息
\echo 'Labels表创建成功！'
\echo '现在可以使用01_data_importer.py重新导入数据，或手动导入labels文件'
