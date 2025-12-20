-- PostGIS 数据库方案：移动测绘系统数据管理
-- 多传感器数据采集和管理系统
-- 使用方法: psql -d your_database -f mapping_system_schema.sql

-- 启用 PostGIS 扩展
CREATE EXTENSION IF NOT EXISTS postgis;

-- ============================================
-- 1. 项目和会话管理
-- ============================================

-- 项目表（顶层组织单位）
CREATE TABLE IF NOT EXISTS projects (
    project_id SERIAL PRIMARY KEY,
    project_name VARCHAR(200) NOT NULL UNIQUE,
    description TEXT,
    location VARCHAR(200),
    source_data_root TEXT,  -- 原始数据根目录路径（可手动指定）
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB
);

-- 数据采集会话表（每次数据采集任务）
CREATE TABLE IF NOT EXISTS data_sessions (
    session_id SERIAL PRIMARY KEY,
    project_id INTEGER REFERENCES projects(project_id) ON DELETE CASCADE,
    session_name VARCHAR(200) NOT NULL,
    session_path TEXT NOT NULL,  -- 文件系统路径，如 /media/.../map_result/1
    source_data_path TEXT,  -- 原始数据路径（可手动指定，指向原始bag文件或数据目录）
    rosbag_file VARCHAR(500),
    start_time TIMESTAMP,
    end_time TIMESTAMP,
    duration_seconds NUMERIC(12, 3),
    message_count INTEGER,
    reference_point GEOMETRY(PointZ, 4326),  -- 参考起点（GPS坐标）
    bbox GEOMETRY(Polygon, 4326),  -- 会话覆盖的地理范围
    description TEXT,
    metadata JSONB,  -- 存储 metadata.yaml 内容
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_path UNIQUE (session_path)
);

-- ============================================
-- 2. 传感器配置
-- ============================================

-- 传感器类型枚举
CREATE TYPE sensor_type_enum AS ENUM ('camera', 'lidar', 'imu', 'gnss', 'other');

-- 传感器表
CREATE TABLE IF NOT EXISTS sensors (
    sensor_id SERIAL PRIMARY KEY,
    sensor_name VARCHAR(100) NOT NULL UNIQUE,  -- cam1, cam2, hesai128, cgi830
    sensor_type sensor_type_enum NOT NULL,
    serial_number VARCHAR(100),
    model VARCHAR(100),
    description TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB
);

-- 传感器内参表
CREATE TABLE IF NOT EXISTS sensor_intrinsics (
    intrinsic_id SERIAL PRIMARY KEY,
    sensor_id INTEGER REFERENCES sensors(sensor_id) ON DELETE CASCADE,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    calibration_time TIMESTAMP,
    image_width INTEGER,
    image_height INTEGER,
    camera_matrix JSONB,  -- 3x3 相机矩阵
    dist_coeffs JSONB,    -- 畸变系数
    model VARCHAR(50),    -- brown, fisheye等
    calibration_data JSONB,  -- 完整的标定数据
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_sensor_session_intrinsic UNIQUE (sensor_id, session_id)
);

-- 传感器外参表（传感器到车体坐标系的转换）
CREATE TABLE IF NOT EXISTS sensor_extrinsics (
    extrinsic_id SERIAL PRIMARY KEY,
    sensor_id INTEGER REFERENCES sensors(sensor_id) ON DELETE CASCADE,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    calibration_time TIMESTAMP,
    rotation_quaternion JSONB,  -- wxyz 四元数
    translation JSONB,           -- xyz 平移向量(米)
    coordinate_system VARCHAR(200),  -- 坐标系说明
    calibration_data JSONB,      -- 完整的标定数据
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_sensor_session_extrinsic UNIQUE (sensor_id, session_id)
);

-- ============================================
-- 3. 位姿数据
-- ============================================

-- 车辆位姿表（主位姿数据）
CREATE TABLE IF NOT EXISTS vehicle_poses (
    pose_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    frame_id INTEGER NOT NULL,  -- 帧ID
    timestamp_sec BIGINT NOT NULL,  -- Unix时间戳（秒）
    timestamp_nsec INTEGER DEFAULT 0,  -- 纳秒部分
    
    -- 局部坐标系位置（通常是UTM或ENU）
    position_x DOUBLE PRECISION NOT NULL,
    position_y DOUBLE PRECISION NOT NULL,
    position_z DOUBLE PRECISION NOT NULL,
    
    -- 旋转矩阵（3x3，行优先存储）
    rotation_matrix DOUBLE PRECISION[9] NOT NULL,
    
    -- GPS 经纬高坐标
    longitude DOUBLE PRECISION,
    latitude DOUBLE PRECISION,
    altitude DOUBLE PRECISION,
    gps_position GEOMETRY(PointZ, 4326),
    
    -- 速度信息
    velocity_x DOUBLE PRECISION,
    velocity_y DOUBLE PRECISION,
    velocity_z DOUBLE PRECISION,
    
    -- 航向角
    heading DOUBLE PRECISION,
    
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_frame UNIQUE (session_id, frame_id)
);

-- 创建位姿索引
CREATE INDEX idx_vehicle_poses_session ON vehicle_poses(session_id);
CREATE INDEX idx_vehicle_poses_timestamp ON vehicle_poses(timestamp_sec);
CREATE INDEX idx_vehicle_poses_gps ON vehicle_poses USING GIST(gps_position);
CREATE INDEX idx_vehicle_poses_frame ON vehicle_poses(frame_id);

-- 优化后的位姿表（稀疏位姿，用于SLAM结果，来自sparse/vehicle_geo_pose）
CREATE TABLE IF NOT EXISTS optimized_poses (
    opt_pose_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    frame_id INTEGER NOT NULL,
    
    -- 时间戳（完整的浮点数时间戳，直接来自YAML的timestamp字段）
    timestamp DOUBLE PRECISION NOT NULL,
    -- 自动计算的秒和纳秒部分（用于与vehicle_poses关联）
    timestamp_sec BIGINT GENERATED ALWAYS AS (FLOOR(timestamp)::BIGINT) STORED,
    timestamp_nsec INTEGER GENERATED ALWAYS AS (((timestamp - FLOOR(timestamp)) * 1e9)::INTEGER) STORED,
    
    -- 4x4 UTM位姿变换矩阵（行优先存储，16个元素，直接来自YAML的pose_utm字段）
    -- 矩阵格式: [R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz, 0, 0, 0, 1]
    -- 其中R是旋转矩阵，T是局部平移向量（相对于offset_utm）
    pose_utm DOUBLE PRECISION[16] NOT NULL,
    
    -- UTM偏移量（全局参考点，直接来自YAML的offset_utm字段）
    offset_utm_x DOUBLE PRECISION NOT NULL,
    offset_utm_y DOUBLE PRECISION NOT NULL,
    offset_utm_z DOUBLE PRECISION NOT NULL,
    
    -- 计算得到的全局UTM坐标（pose_utm中的平移部分 + offset_utm）
    -- 注意：PostgreSQL数组索引从1开始
    -- pose_utm[4] 是 Tx, pose_utm[8] 是 Ty, pose_utm[12] 是 Tz
    position_utm_x DOUBLE PRECISION GENERATED ALWAYS AS (pose_utm[4] + offset_utm_x) STORED,
    position_utm_y DOUBLE PRECISION GENERATED ALWAYS AS (pose_utm[8] + offset_utm_y) STORED,
    position_utm_z DOUBLE PRECISION GENERATED ALWAYS AS (pose_utm[12] + offset_utm_z) STORED,
    
    -- GPS坐标（可选，通过UTM转WGS84计算得到）
    longitude DOUBLE PRECISION,
    latitude DOUBLE PRECISION,
    altitude DOUBLE PRECISION,
    gps_position GEOMETRY(PointZ, 4326),
    
    -- 源文件路径（用于追溯YAML文件）
    yaml_file_path TEXT,
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_opt_session_frame UNIQUE (session_id, frame_id)
);

-- 创建索引
CREATE INDEX idx_optimized_poses_session ON optimized_poses(session_id);
CREATE INDEX idx_optimized_poses_timestamp ON optimized_poses(timestamp);
CREATE INDEX idx_optimized_poses_timestamp_sec ON optimized_poses(timestamp_sec);
CREATE INDEX idx_optimized_poses_gps ON optimized_poses USING GIST(gps_position);
-- 复合索引用于空间查询（二维平面查询）
CREATE INDEX idx_optimized_poses_utm_xy ON optimized_poses(position_utm_x, position_utm_y);
-- 复合索引用于三维空间查询
CREATE INDEX idx_optimized_poses_utm_xyz ON optimized_poses(position_utm_x, position_utm_y, position_utm_z);

-- ============================================
-- 4. 图像数据
-- ============================================

-- 图像表
CREATE TABLE IF NOT EXISTS images (
    image_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    pose_id BIGINT REFERENCES vehicle_poses(pose_id) ON DELETE CASCADE,
    sensor_id INTEGER REFERENCES sensors(sensor_id),
    
    frame_id INTEGER NOT NULL,
    timestamp_sec BIGINT NOT NULL,
    timestamp_nsec INTEGER DEFAULT 0,
    
    image_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,  -- 相对于 session_path 的路径
    absolute_path TEXT NOT NULL,  -- 完整的文件系统路径
    
    width INTEGER,
    height INTEGER,
    format VARCHAR(20),  -- jpg, png, etc.
    file_size_bytes BIGINT,
    
    -- 图像采集位置（继承自pose）
    capture_position GEOMETRY(PointZ, 4326),
    
    thumbnail BYTEA,  -- 缩略图（可选）
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_sensor_frame_image UNIQUE (session_id, sensor_id, frame_id)
);

CREATE INDEX idx_images_session ON images(session_id);
CREATE INDEX idx_images_pose ON images(pose_id);
CREATE INDEX idx_images_sensor ON images(sensor_id);
CREATE INDEX idx_images_timestamp ON images(timestamp_sec);
CREATE INDEX idx_images_position ON images USING GIST(capture_position);

-- ============================================
-- 5. 点云数据
-- ============================================

-- 点云表
CREATE TABLE IF NOT EXISTS point_clouds (
    cloud_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    pose_id BIGINT REFERENCES vehicle_poses(pose_id) ON DELETE CASCADE,
    sensor_id INTEGER REFERENCES sensors(sensor_id),
    
    frame_id INTEGER NOT NULL,
    timestamp_sec BIGINT NOT NULL,
    timestamp_nsec INTEGER DEFAULT 0,
    
    cloud_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,  -- 相对路径
    absolute_path TEXT NOT NULL,  -- 完整路径
    
    format VARCHAR(20),  -- pcd, ply, las, etc.
    point_count BIGINT,
    file_size_bytes BIGINT,
    
    -- 点云边界框（局部坐标系）
    bbox_min_x DOUBLE PRECISION,
    bbox_min_y DOUBLE PRECISION,
    bbox_min_z DOUBLE PRECISION,
    bbox_max_x DOUBLE PRECISION,
    bbox_max_y DOUBLE PRECISION,
    bbox_max_z DOUBLE PRECISION,
    
    -- 中心点
    centroid_x DOUBLE PRECISION,
    centroid_y DOUBLE PRECISION,
    centroid_z DOUBLE PRECISION,
    centroid_geo GEOMETRY(PointZ, 4326),  -- 地理坐标中心点
    
    has_color BOOLEAN DEFAULT FALSE,
    has_intensity BOOLEAN DEFAULT FALSE,
    has_normal BOOLEAN DEFAULT FALSE,
    has_ring BOOLEAN DEFAULT FALSE,  -- 激光雷达ring信息
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_sensor_frame_cloud UNIQUE (session_id, sensor_id, frame_id)
);

CREATE INDEX idx_point_clouds_session ON point_clouds(session_id);
CREATE INDEX idx_point_clouds_pose ON point_clouds(pose_id);
CREATE INDEX idx_point_clouds_sensor ON point_clouds(sensor_id);
CREATE INDEX idx_point_clouds_timestamp ON point_clouds(timestamp_sec);
CREATE INDEX idx_point_clouds_centroid ON point_clouds USING GIST(centroid_geo);

-- ============================================
-- 6. 深度图数据
-- ============================================

-- 深度图表
CREATE TABLE IF NOT EXISTS depth_maps (
    depth_id BIGSERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    pose_id BIGINT REFERENCES vehicle_poses(pose_id) ON DELETE CASCADE,
    sensor_id INTEGER REFERENCES sensors(sensor_id),
    
    frame_id INTEGER NOT NULL,
    timestamp_sec BIGINT NOT NULL,
    
    depth_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,
    absolute_path TEXT NOT NULL,
    
    width INTEGER,
    height INTEGER,
    format VARCHAR(20),
    file_size_bytes BIGINT,
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_sensor_frame_depth UNIQUE (session_id, sensor_id, frame_id)
);

CREATE INDEX idx_depth_maps_session ON depth_maps(session_id);
CREATE INDEX idx_depth_maps_pose ON depth_maps(pose_id);

-- ============================================
-- 7. 地图重建数据
-- ============================================

-- 子地图表（SLAM生成的局部地图）
CREATE TABLE IF NOT EXISTS submaps (
    submap_id SERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    
    submap_name VARCHAR(255) NOT NULL,
    submap_index INTEGER NOT NULL,  -- 如 submap_3_5 的第一个数字
    submap_version INTEGER,          -- 如 submap_3_5 的第二个数字
    
    relative_path TEXT NOT NULL,
    absolute_path TEXT NOT NULL,
    
    format VARCHAR(20),  -- pcd, ply, etc.
    point_count BIGINT,
    file_size_bytes BIGINT,
    
    -- 子地图覆盖范围
    bbox_min_x DOUBLE PRECISION,
    bbox_min_y DOUBLE PRECISION,
    bbox_min_z DOUBLE PRECISION,
    bbox_max_x DOUBLE PRECISION,
    bbox_max_y DOUBLE PRECISION,
    bbox_max_z DOUBLE PRECISION,
    bbox_geo GEOMETRY(PolygonZ, 4326),  -- 地理坐标范围
    
    centroid_geo GEOMETRY(PointZ, 4326),
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_submap UNIQUE (session_id, submap_name)
);

CREATE INDEX idx_submaps_session ON submaps(session_id);
CREATE INDEX idx_submaps_bbox ON submaps USING GIST(bbox_geo);
CREATE INDEX idx_submaps_centroid ON submaps USING GIST(centroid_geo);

-- 轨迹表（完整的优化轨迹）
CREATE TABLE IF NOT EXISTS trajectories (
    trajectory_id SERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    
    trajectory_name VARCHAR(255) NOT NULL,
    relative_path TEXT NOT NULL,
    absolute_path TEXT NOT NULL,
    
    format VARCHAR(20),  -- ply, etc.
    point_count BIGINT,
    
    -- 轨迹的地理范围
    trajectory_line GEOMETRY(LineStringZ, 4326),
    bbox GEOMETRY(PolygonZ, 4326),
    
    total_distance_meters DOUBLE PRECISION,
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_trajectory UNIQUE (session_id, trajectory_name)
);

CREATE INDEX idx_trajectories_session ON trajectories(session_id);
CREATE INDEX idx_trajectories_line ON trajectories USING GIST(trajectory_line);

-- ============================================
-- 8. ROS2 话题信息
-- ============================================

-- ROS2话题表
CREATE TABLE IF NOT EXISTS ros2_topics (
    topic_id SERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES data_sessions(session_id) ON DELETE CASCADE,
    
    topic_name VARCHAR(500) NOT NULL,
    message_type VARCHAR(200),
    message_count INTEGER,
    serialization_format VARCHAR(50),
    
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_session_topic UNIQUE (session_id, topic_name)
);

CREATE INDEX idx_ros2_topics_session ON ros2_topics(session_id);

-- ============================================
-- 9. 视图：方便查询
-- ============================================

-- 会话统计视图
CREATE OR REPLACE VIEW session_statistics AS
SELECT 
    ds.session_id,
    ds.session_name,
    ds.project_id,
    COUNT(DISTINCT vp.pose_id) as total_poses,
    COUNT(DISTINCT i.image_id) as total_images,
    COUNT(DISTINCT pc.cloud_id) as total_point_clouds,
    COUNT(DISTINCT sm.submap_id) as total_submaps,
    MIN(vp.timestamp_sec) as first_timestamp,
    MAX(vp.timestamp_sec) as last_timestamp,
    ST_Extent(vp.gps_position) as trajectory_extent
FROM data_sessions ds
LEFT JOIN vehicle_poses vp ON ds.session_id = vp.session_id
LEFT JOIN images i ON ds.session_id = i.session_id
LEFT JOIN point_clouds pc ON ds.session_id = pc.session_id
LEFT JOIN submaps sm ON ds.session_id = sm.session_id
GROUP BY ds.session_id, ds.session_name, ds.project_id;

-- 传感器数据统计视图
CREATE OR REPLACE VIEW sensor_data_statistics AS
SELECT 
    s.sensor_id,
    s.sensor_name,
    s.sensor_type,
    ds.session_id,
    ds.session_name,
    COUNT(DISTINCT i.image_id) as image_count,
    COUNT(DISTINCT pc.cloud_id) as cloud_count
FROM sensors s
CROSS JOIN data_sessions ds
LEFT JOIN images i ON s.sensor_id = i.sensor_id AND ds.session_id = i.session_id
LEFT JOIN point_clouds pc ON s.sensor_id = pc.sensor_id AND ds.session_id = pc.session_id
GROUP BY s.sensor_id, s.sensor_name, s.sensor_type, ds.session_id, ds.session_name;

-- ============================================
-- 10. 辅助函数
-- ============================================

-- 计算两个位姿之间的距离（米）
CREATE OR REPLACE FUNCTION calculate_pose_distance(
    x1 DOUBLE PRECISION, y1 DOUBLE PRECISION, z1 DOUBLE PRECISION,
    x2 DOUBLE PRECISION, y2 DOUBLE PRECISION, z2 DOUBLE PRECISION
)
RETURNS DOUBLE PRECISION AS $$
BEGIN
    RETURN SQRT(POWER(x2 - x1, 2) + POWER(y2 - y1, 2) + POWER(z2 - z1, 2));
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- 从时间戳创建TIMESTAMP
CREATE OR REPLACE FUNCTION timestamp_from_unix(
    sec BIGINT,
    nsec INTEGER DEFAULT 0
)
RETURNS TIMESTAMP AS $$
BEGIN
    RETURN to_timestamp(sec + nsec / 1000000000.0);
END;
$$ LANGUAGE plpgsql IMMUTABLE;

-- ============================================
-- 11. 注释
-- ============================================

COMMENT ON TABLE projects IS '项目表，组织多个数据采集会话';
COMMENT ON TABLE data_sessions IS '数据采集会话，对应一次完整的数据采集任务';
COMMENT ON TABLE sensors IS '传感器配置表';
COMMENT ON TABLE sensor_intrinsics IS '传感器内参标定数据';
COMMENT ON TABLE sensor_extrinsics IS '传感器外参标定数据（传感器到车体）';
COMMENT ON TABLE vehicle_poses IS '车辆位姿数据（主要定位数据源）';
COMMENT ON TABLE optimized_poses IS 'SLAM优化后的稀疏位姿，来自sparse/vehicle_geo_pose/*.yaml文件';
COMMENT ON TABLE images IS '图像数据索引';
COMMENT ON TABLE point_clouds IS '点云数据索引';
COMMENT ON TABLE depth_maps IS '深度图数据索引';
COMMENT ON TABLE submaps IS 'SLAM生成的局部子地图';
COMMENT ON TABLE trajectories IS '优化后的完整轨迹';
COMMENT ON TABLE ros2_topics IS 'ROS2话题信息';

-- 为数据来源字段添加注释
COMMENT ON COLUMN projects.source_data_root IS '项目原始数据根目录路径，可手动指定';
COMMENT ON COLUMN data_sessions.source_data_path IS '会话原始数据路径，可手动指定（如原始bag文件或采集数据目录）';

-- 为optimized_poses表的重要字段添加注释
COMMENT ON COLUMN optimized_poses.timestamp IS '完整的浮点数时间戳（秒），来自YAML文件';
COMMENT ON COLUMN optimized_poses.pose_utm IS '4x4 UTM位姿变换矩阵，行优先存储（相对于offset_utm的局部坐标）';
COMMENT ON COLUMN optimized_poses.offset_utm_x IS 'UTM东向偏移量（米），全局参考点X坐标';
COMMENT ON COLUMN optimized_poses.offset_utm_y IS 'UTM北向偏移量（米），全局参考点Y坐标';
COMMENT ON COLUMN optimized_poses.offset_utm_z IS 'UTM高程偏移量（米），全局参考点Z坐标';
COMMENT ON COLUMN optimized_poses.position_utm_x IS '计算得到的全局UTM X坐标 = pose_utm[4] + offset_utm_x';
COMMENT ON COLUMN optimized_poses.position_utm_y IS '计算得到的全局UTM Y坐标 = pose_utm[8] + offset_utm_y';
COMMENT ON COLUMN optimized_poses.position_utm_z IS '计算得到的全局UTM Z坐标 = pose_utm[12] + offset_utm_z';

-- ============================================
-- 完成
-- ============================================

SELECT 'Mapping System Database Schema Created Successfully!' as status;
