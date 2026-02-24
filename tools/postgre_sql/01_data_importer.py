#!/usr/bin/env python3
"""
移动测绘系统数据导入工具
从文件系统导入传感器数据到PostgreSQL/PostGIS数据库
"""

import os
import sys
import yaml
import json
import psycopg2
from psycopg2.extras import execute_batch
from pathlib import Path
from datetime import datetime
import numpy as np
from typing import Dict, List, Tuple, Optional
import logging
from qt.load_config import load_config

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# 添加OpenCV YAML标签的构造器
def opencv_matrix_constructor(loader, node):
    """处理OpenCV矩阵标签"""
    mapping = loader.construct_mapping(node, deep=True)
    return mapping

# 注册OpenCV标签构造器到FullLoader（更宽松的加载器）
yaml.FullLoader.add_constructor('tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)
yaml.FullLoader.add_constructor('!!opencv-matrix', opencv_matrix_constructor)


def load_opencv_yaml(file_path):
    """
    加载OpenCV格式的YAML文件
    处理%YAML:1.0指令和!!opencv-matrix标签
    """
    with open(file_path, 'r') as f:
        content = f.read()
        # 移除%YAML:1.0指令（如果存在）
        if content.startswith('%YAML'):
            lines = content.split('\n')
            content = '\n'.join(lines[1:])  # 跳过第一行
        return yaml.load(content, Loader=yaml.FullLoader)


class MappingDataImporter:
    """移动测绘数据导入器"""

    def __init__(self, db_config: Dict[str, str]):
        """
        初始化导入器

        Args:
            db_config: 数据库连接配置
                - host: 主机地址
                - port: 端口
                - database: 数据库名
                - user: 用户名
                - password: 密码
        """
        self.db_config = db_config
        self.conn = None
        self.cursor = None
        # savepoint counter for safe batch operations
        self._sp_counter = 0
        
    def connect(self):
        """连接数据库"""
        try:
            self.conn = psycopg2.connect(**self.db_config)
            self.cursor = self.conn.cursor()
            logger.info("数据库连接成功")
        except Exception as e:
            logger.error(f"数据库连接失败: {e}")
            raise
    
    def close(self):
        """关闭数据库连接"""
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
        logger.info("数据库连接已关闭")
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            self.conn.commit()
        else:
            self.conn.rollback()
            logger.error(f"事务回滚: {exc_val}")
        self.close()
    
    def create_or_get_project(self, project_name: str, description: str = None) -> int:
        """创建或获取项目ID"""
        # 尝试获取已存在的项目
        self.cursor.execute(
            "SELECT project_id FROM projects WHERE project_name = %s",
            (project_name,)
        )
        result = self.cursor.fetchone()
        
        if result:
            project_id = result[0]
            logger.info(f"使用已存在的项目: {project_name} (ID: {project_id})")
        else:
            # 创建新项目
            self.cursor.execute(
                """
                INSERT INTO projects (project_name, description)
                VALUES (%s, %s)
                RETURNING project_id
                """,
                (project_name, description)
            )
            project_id = self.cursor.fetchone()[0]
            self.conn.commit()
            logger.info(f"创建新项目: {project_name} (ID: {project_id})")
        
        return project_id
    
    def import_sensors(self, calib_dir: Path) -> Dict[str, int]:
        """
        导入传感器配置
        
        Args:
            calib_dir: 标定目录路径
            
        Returns:
            传感器名称到ID的映射
        """
        sensor_mapping = {}
        
        # 从内参文件获取相机列表
        intrinsics_dir = calib_dir / "intrinsics"
        if intrinsics_dir.exists():
            for intrinsic_file in intrinsics_dir.glob("*_Intrinsic.yaml"):
                # 解析文件名获取传感器名称，如 cam1_Intrinsic.yaml -> cam1
                sensor_name = intrinsic_file.stem.replace("_Intrinsic", "")
                
                # 检查传感器是否已存在
                self.cursor.execute(
                    "SELECT sensor_id FROM sensors WHERE sensor_name = %s",
                    (sensor_name,)
                )
                result = self.cursor.fetchone()
                
                if result:
                    sensor_mapping[sensor_name] = result[0]
                else:
                    # 创建新传感器
                    self.cursor.execute(
                        """
                        INSERT INTO sensors (sensor_name, sensor_type)
                        VALUES (%s, %s)
                        RETURNING sensor_id
                        """,
                        (sensor_name, 'camera')
                    )
                    sensor_mapping[sensor_name] = self.cursor.fetchone()[0]
                    logger.info(f"创建传感器: {sensor_name}")
        
        # 从外参文件获取其他传感器（激光雷达、IMU等）
        extrinsics_dir = calib_dir / "extrinsics"
        if extrinsics_dir.exists():
            for extrinsic_file in extrinsics_dir.glob("*_Extrinsics.yaml"):
                # 解析文件名，如 0_hesai128_2_vehicle_Extrinsics.yaml
                parts = extrinsic_file.stem.split("_")
                if len(parts) >= 2:
                    sensor_name = parts[1]  # hesai128, cgi830等
                    
                    if sensor_name not in sensor_mapping:
                        # 判断传感器类型
                        sensor_type = 'other'
                        if 'hesai' in sensor_name.lower() or 'lidar' in sensor_name.lower():
                            sensor_type = 'lidar'
                        elif 'cgi' in sensor_name.lower() or 'imu' in sensor_name.lower():
                            sensor_type = 'imu'
                        
                        self.cursor.execute(
                            "SELECT sensor_id FROM sensors WHERE sensor_name = %s",
                            (sensor_name,)
                        )
                        result = self.cursor.fetchone()
                        
                        if result:
                            sensor_mapping[sensor_name] = result[0]
                        else:
                            self.cursor.execute(
                                """
                                INSERT INTO sensors (sensor_name, sensor_type)
                                VALUES (%s, %s::sensor_type_enum)
                                RETURNING sensor_id
                                """,
                                (sensor_name, sensor_type)
                            )
                            sensor_mapping[sensor_name] = self.cursor.fetchone()[0]
                            logger.info(f"创建传感器: {sensor_name} (类型: {sensor_type})")
        
        self.conn.commit()
        return sensor_mapping
    
    def import_session(self, session_path: Path, project_id: int, 
                      session_name: str = None) -> Tuple[int, bool]:
        """
        导入数据采集会话
        
        Args:
            session_path: 会话数据目录路径
            project_id: 项目ID
            session_name: 会话名称（可选，默认使用目录名）
            
        Returns:
            (session_id, is_new): 会话ID和是否为新会话的标志
        """
        if session_name is None:
            session_name = session_path.name
        
        # 读取metadata.yaml
        metadata_file = session_path / "metadata.yaml"
        metadata = {}
        start_time = None
        end_time = None
        duration_seconds = None
        message_count = None
        
        if metadata_file.exists():
            # metadata.yaml 使用标准YAML格式
            with open(metadata_file, 'r') as f:
                metadata = yaml.safe_load(f)
                
            # 提取关键信息
            bag_info = metadata.get('rosbag2_bagfile_information', {})
            duration_ns = bag_info.get('duration', {}).get('nanoseconds', 0)
            duration_seconds = duration_ns / 1e9
            message_count = bag_info.get('message_count', 0)
            
            start_ns = bag_info.get('starting_time', {}).get('nanoseconds_since_epoch', 0)
            if start_ns > 0:
                start_time = datetime.fromtimestamp(start_ns / 1e9)
                end_time = datetime.fromtimestamp((start_ns + duration_ns) / 1e9)
        
        # 检查会话是否已存在
        self.cursor.execute(
            "SELECT session_id FROM data_sessions WHERE session_path = %s",
            (str(session_path),)
        )
        result = self.cursor.fetchone()
        
        if result:
            session_id = result[0]
            logger.info(f"会话已存在，跳过: {session_name} (ID: {session_id})")
            return session_id, False  # 返回False表示不是新会话
        
        # 创建新会话
        self.cursor.execute(
            """
            INSERT INTO data_sessions 
            (project_id, session_name, session_path, start_time, end_time, 
             duration_seconds, message_count, metadata)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
            RETURNING session_id
            """,
            (project_id, session_name, str(session_path), start_time, end_time,
             duration_seconds, message_count, json.dumps(metadata))
        )
        session_id = self.cursor.fetchone()[0]
        self.conn.commit()
        
        logger.info(f"创建新会话: {session_name} (ID: {session_id})")
        return session_id, True  # 返回True表示是新会话
    
    def import_calibrations(self, session_path: Path, session_id: int, 
                           sensor_mapping: Dict[str, int]):
        """导入标定数据"""
        calib_dir = session_path / "calib"
        
        # 导入内参
        intrinsics_dir = calib_dir / "intrinsics"
        if intrinsics_dir.exists():
            for intrinsic_file in intrinsics_dir.glob("*_Intrinsic.yaml"):
                sensor_name = intrinsic_file.stem.replace("_Intrinsic", "")
                if sensor_name not in sensor_mapping:
                    continue
                
                sensor_id = sensor_mapping[sensor_name]
                
                # 使用load_opencv_yaml处理OpenCV格式的YAML文件
                calib_data = load_opencv_yaml(intrinsic_file)
                
                # 提取关键信息
                calib_time_str = calib_data.get('calibration_time', '')
                calib_time = None
                if calib_time_str:
                    try:
                        calib_time = datetime.strptime(calib_time_str, '%Y_%m_%d %H:%M:%S')
                    except:
                        pass
                
                # 检查是否已存在
                self.cursor.execute(
                    """
                    SELECT intrinsic_id FROM sensor_intrinsics 
                    WHERE sensor_id = %s AND session_id = %s
                    """,
                    (sensor_id, session_id)
                )
                
                if self.cursor.fetchone():
                    continue  # 已存在，跳过
                
                self.cursor.execute(
                    """
                    INSERT INTO sensor_intrinsics
                    (sensor_id, session_id, calibration_time, image_width, image_height,
                     camera_matrix, dist_coeffs, model, calibration_data)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
                    """,
                    (sensor_id, session_id, calib_time,
                     calib_data.get('ImageWidth'),
                     calib_data.get('ImageHeight'),
                     json.dumps(calib_data.get('cameraMatrix')),
                     json.dumps(calib_data.get('distCoeffs')),
                     calib_data.get('Model'),
                     json.dumps(calib_data))
                )
                logger.info(f"导入内参: {sensor_name}")
        
        # 导入外参
        extrinsics_dir = calib_dir / "extrinsics"
        if extrinsics_dir.exists():
            for extrinsic_file in extrinsics_dir.glob("*_Extrinsics.yaml"):
                parts = extrinsic_file.stem.split("_")
                if len(parts) >= 2:
                    sensor_name = parts[1]
                    if sensor_name not in sensor_mapping:
                        continue
                    
                    sensor_id = sensor_mapping[sensor_name]
                    
                    # 使用load_opencv_yaml处理OpenCV格式的YAML文件
                    calib_data = load_opencv_yaml(extrinsic_file)
                    
                    calib_time_str = calib_data.get('calibration_time', '')
                    calib_time = None
                    if calib_time_str:
                        try:
                            calib_time = datetime.strptime(calib_time_str, '%Y_%m_%d %H:%M:%S')
                        except:
                            pass
                    
                    # 检查是否已存在
                    self.cursor.execute(
                        """
                        SELECT extrinsic_id FROM sensor_extrinsics 
                        WHERE sensor_id = %s AND session_id = %s
                        """,
                        (sensor_id, session_id)
                    )
                    
                    if self.cursor.fetchone():
                        continue
                    
                    self.cursor.execute(
                        """
                        INSERT INTO sensor_extrinsics
                        (sensor_id, session_id, calibration_time, rotation_quaternion,
                         translation, coordinate_system, calibration_data)
                        VALUES (%s, %s, %s, %s, %s, %s, %s)
                        """,
                        (sensor_id, session_id, calib_time,
                         json.dumps(calib_data.get('r_quaternion_wxyz')),
                         json.dumps(calib_data.get('t_metric_xyz')),
                         calib_data.get('vehicle_coordinate_system'),
                         json.dumps(calib_data))
                    )
                    logger.info(f"导入外参: {sensor_name}")
        
        self.conn.commit()
    
    def import_odoms(self, session_path: Path, session_id: int, batch_size: int = 1000):
        """
        导入odom位姿数据到vehicle_poses表
        
        Args:
            session_path: 会话路径
            session_id: 会话ID
            batch_size: 批量插入大小
        """
        odoms_dir = session_path / "odoms"
        if not odoms_dir.exists():
            logger.warning(f"odoms目录不存在: {odoms_dir}")
            return
        
        # 检查该session的odom数据是否已存在
        self.cursor.execute(
            "SELECT COUNT(*) FROM vehicle_poses WHERE session_id = %s",
            (session_id,)
        )
        existing_count = self.cursor.fetchone()[0]
        if existing_count > 0:
            logger.info(f"odoms数据已存在({existing_count}条)，跳过导入")
            return
        
        # 获取所有位姿文件并排序
        odom_files = sorted(odoms_dir.glob("*.yaml"))
        logger.info(f"找到 {len(odom_files)} 个odom位姿文件")
        
        poses_data = []
        for odom_file in odom_files:
            # 解析文件名：0_1763951779.000.yaml
            parts = odom_file.stem.split("_")
            frame_id = int(parts[0])
            timestamp_sec = int(float(parts[1]))
            timestamp_nsec = int((float(parts[1]) - timestamp_sec) * 1e9)
            
            # 使用load_opencv_yaml处理OpenCV格式的YAML文件
            odom_data = load_opencv_yaml(odom_file)
            
            # 提取数据
            position = odom_data.get('position', [0, 0, 0])
            pose_matrix = odom_data.get('pose', [])  # 3x3旋转矩阵（行优先）
            position_lla = odom_data.get('position_lla', [])  # [lat, lon, alt]
            velocity = odom_data.get('enu_velocity', [0, 0, 0])
            heading = odom_data.get('heading', 0)
            
            # 准备插入数据
            pose_row = (
                session_id,
                frame_id,
                timestamp_sec,
                timestamp_nsec,
                position[0], position[1], position[2],
                pose_matrix,
                position_lla[1] if len(position_lla) >= 2 else None,  # longitude
                position_lla[0] if len(position_lla) >= 1 else None,  # latitude
                position_lla[2] if len(position_lla) >= 3 else None,  # altitude
                velocity[0], velocity[1], velocity[2],
                heading
            )
            poses_data.append(pose_row)
            
            # 批量插入
            if len(poses_data) >= batch_size:
                self._batch_insert_poses(poses_data)
                poses_data = []
        
        # 插入剩余数据
        if poses_data:
            self._batch_insert_poses(poses_data)
        
        logger.info(f"导入odom位姿数据完成: {len(odom_files)} 条")
    
    def import_poses(self, session_path: Path, session_id: int, batch_size: int = 1000):
        """
        导入优化后的位姿数据到optimized_poses表（来自sparse/vehicle_geo_pose）
        
        新表结构说明：
        - timestamp: 完整的浮点数时间戳（直接来自YAML）
        - timestamp_sec/nsec: 自动计算（GENERATED ALWAYS）
        - pose_utm: 4x4矩阵（16个元素）
        - offset_utm_x/y/z: UTM偏移量
        - position_utm_x/y/z: 自动计算（GENERATED ALWAYS）
        
        Args:
            session_path: 会话路径
            session_id: 会话ID
            batch_size: 批量插入大小
        """
        poses_dir = session_path / "sparse" / "vehicle_geo_pose"
        if not poses_dir.exists():
            logger.warning(f"优化位姿目录不存在: {poses_dir}")
            return
        
        # 检查该session的优化位姿数据是否已存在
        self.cursor.execute(
            "SELECT COUNT(*) FROM optimized_poses WHERE session_id = %s",
            (session_id,)
        )
        existing_count = self.cursor.fetchone()[0]
        if existing_count > 0:
            logger.info(f"优化位姿数据已存在({existing_count}条)，跳过导入")
            return
        
        # 获取所有位姿文件并排序
        pose_files = sorted(poses_dir.glob("*.yaml"))
        logger.info(f"找到 {len(pose_files)} 个优化位姿文件")
        
        poses_data = []
        for pose_file in pose_files:
            try:
                # 使用load_opencv_yaml处理YAML文件
                pose_data = load_opencv_yaml(pose_file)
                
                # 提取时间戳（直接从YAML读取完整的浮点数时间戳）
                timestamp = pose_data.get('timestamp', None)
                if timestamp is None:
                    # 如果YAML中没有timestamp字段，从文件名解析
                    parts = pose_file.stem.split("_")
                    frame_id = int(parts[0])
                    timestamp = float(parts[1])
                else:
                    # 从文件名获取frame_id
                    parts = pose_file.stem.split("_")
                    frame_id = int(parts[0])
                
                # 提取数据
                pose_utm = pose_data.get('pose_utm', [])  # 4x4矩阵，行优先存储
                offset_utm = pose_data.get('offset_utm', [0, 0, 0])
                
                if len(pose_utm) != 16:
                    logger.warning(f"位姿矩阵格式错误（需要16个元素）: {pose_file.name}")
                    continue
                
                # 确保offset_utm有3个元素
                offset_utm_x = offset_utm[0] if len(offset_utm) >= 1 else 0.0
                offset_utm_y = offset_utm[1] if len(offset_utm) >= 2 else 0.0
                offset_utm_z = offset_utm[2] if len(offset_utm) >= 3 else 0.0
                
                # 准备插入数据
                # 注意：position_utm_x/y/z 和 timestamp_sec/nsec 由数据库自动计算
                pose_row = (
                    session_id,
                    frame_id,
                    timestamp,          # 完整的浮点数时间戳
                    pose_utm,           # 4x4矩阵（16个元素）
                    offset_utm_x,
                    offset_utm_y,
                    offset_utm_z,
                    str(pose_file)      # yaml文件路径
                )
                poses_data.append(pose_row)
                
                # 批量插入
                if len(poses_data) >= batch_size:
                    self._batch_insert_optimized_poses(poses_data)
                    poses_data = []
                    
            except Exception as e:
                logger.error(f"处理文件失败 {pose_file.name}: {e}")
                continue
        
        # 插入剩余数据
        if poses_data:
            self._batch_insert_optimized_poses(poses_data)
        
        logger.info(f"导入优化位姿数据完成: {len(pose_files)} 条")
    
    def _batch_insert_poses(self, poses_data: List[Tuple]):
        """批量插入位姿数据到vehicle_poses表"""
        query = """
            INSERT INTO vehicle_poses
            (session_id, frame_id, timestamp_sec, timestamp_nsec,
             position_x, position_y, position_z, rotation_matrix,
             longitude, latitude, altitude, gps_position,
             velocity_x, velocity_y, velocity_z, heading)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, 
                    CASE WHEN %s IS NOT NULL AND %s IS NOT NULL 
                         THEN ST_SetSRID(ST_MakePoint(%s, %s, COALESCE(%s, 0)), 4326)
                         ELSE NULL 
                    END,
                    %s, %s, %s, %s)
            ON CONFLICT (session_id, frame_id) DO NOTHING
        """
        params = [
            (row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7],
             row[8], row[9], row[10],
             row[8], row[9], row[8], row[9], row[10],
             row[11], row[12], row[13], row[14])
            for row in poses_data
        ]
        # Use safe helper to avoid leaving transaction aborted
        ok = self._safe_execute_batch(query, params, desc="vehicle_poses")
        if not ok:
            logger.error("部分位姿批量插入失败，已跳过该批次")
    
    def _batch_insert_optimized_poses(self, poses_data: List[Tuple]):
        """
        批量插入优化位姿数据到optimized_poses表
        
        新表结构字段说明：
        - session_id, frame_id: 基本标识
        - timestamp: 完整的浮点数时间戳（YAML中的timestamp字段）
        - pose_utm: 4x4矩阵数组（16个元素）
        - offset_utm_x, offset_utm_y, offset_utm_z: UTM偏移量
        - yaml_file_path: 源文件路径
        
        自动计算字段（不需要插入）：
        - timestamp_sec, timestamp_nsec: 由timestamp自动计算
        - position_utm_x, position_utm_y, position_utm_z: 由pose_utm和offset_utm自动计算
        """
        query = """
            INSERT INTO optimized_poses
            (session_id, frame_id, timestamp, pose_utm,
             offset_utm_x, offset_utm_y, offset_utm_z, yaml_file_path)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
            ON CONFLICT (session_id, frame_id) DO NOTHING
        """
        ok = self._safe_execute_batch(query, poses_data, desc="optimized_poses")
        if not ok:
            logger.error("部分优化位姿批量插入失败，已跳过该批次")
    
    def import_images(self, session_path: Path, session_id: int, 
                     sensor_mapping: Dict[str, int], batch_size: int = 500):
        """导入图像数据索引"""
        images_dir = session_path / "images"
        if not images_dir.exists():
            logger.warning(f"图像目录不存在: {images_dir}")
            return
        
        # 检查该session的图像数据是否已存在
        self.cursor.execute(
            "SELECT COUNT(*) FROM images WHERE session_id = %s",
            (session_id,)
        )
        existing_count = self.cursor.fetchone()[0]
        if existing_count > 0:
            logger.info(f"图像数据已存在({existing_count}条)，跳过导入")
            return
        
        # 遍历各个相机目录
        for cam_dir in images_dir.iterdir():
            if not cam_dir.is_dir():
                continue
            
            sensor_name = cam_dir.name  # cam1, cam2, etc.
            if sensor_name not in sensor_mapping:
                logger.warning(f"未知传感器: {sensor_name}")
                continue
            
            sensor_id = sensor_mapping[sensor_name]
            image_files = sorted(cam_dir.glob("*.jpg")) + sorted(cam_dir.glob("*.png"))
            
            logger.info(f"处理 {sensor_name}: {len(image_files)} 张图像")
            
            images_data = []
            for img_file in image_files:
                # 解析文件名：0_1763951779.000_cam1.jpg
                parts = img_file.stem.split("_")
                frame_id = int(parts[0])
                timestamp_sec = int(float(parts[1]))
                timestamp_nsec = int((float(parts[1]) - timestamp_sec) * 1e9)
                
                # 获取关联的pose_id
                self.cursor.execute(
                    """
                    SELECT pose_id FROM vehicle_poses 
                    WHERE session_id = %s AND frame_id = %s
                    """,
                    (session_id, frame_id)
                )
                result = self.cursor.fetchone()
                pose_id = result[0] if result else None
                
                relative_path = str(img_file.relative_to(session_path))
                file_size = img_file.stat().st_size if img_file.exists() else 0
                
                images_data.append((
                    session_id, pose_id, sensor_id,
                    frame_id, timestamp_sec, timestamp_nsec,
                    img_file.name, relative_path, str(img_file),
                    img_file.suffix.lstrip('.'), file_size
                ))
                
                if len(images_data) >= batch_size:
                    self._batch_insert_images(images_data)
                    images_data = []
            
            if images_data:
                self._batch_insert_images(images_data)
        
        logger.info("导入图像索引完成")
    
    def _batch_insert_images(self, images_data: List[Tuple]):
        """批量插入图像数据"""
        query = """
            INSERT INTO images
            (session_id, pose_id, sensor_id, frame_id, timestamp_sec, timestamp_nsec,
             image_name, relative_path, absolute_path, format, file_size_bytes)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            ON CONFLICT (session_id, sensor_id, frame_id) DO NOTHING
        """
        ok = self._safe_execute_batch(query, images_data, desc="images")
        if not ok:
            logger.error("部分图像批量插入失败，已跳过该批次")
    
    def import_point_clouds(self, session_path: Path, session_id: int,
                           sensor_mapping: Dict[str, int], batch_size: int = 500):
        """导入点云数据索引"""
        pointclouds_dir = session_path / "pointclouds"
        if not pointclouds_dir.exists():
            logger.warning(f"点云目录不存在: {pointclouds_dir}")
            return
        
        # 检查该session的点云数据是否已存在
        self.cursor.execute(
            "SELECT COUNT(*) FROM point_clouds WHERE session_id = %s",
            (session_id,)
        )
        existing_count = self.cursor.fetchone()[0]
        if existing_count > 0:
            logger.info(f"点云数据已存在({existing_count}条)，跳过导入")
            return
        
        # 假设点云来自某个激光雷达传感器
        lidar_sensor_id = None
        for name, sid in sensor_mapping.items():
            if 'hesai' in name.lower() or 'lidar' in name.lower():
                lidar_sensor_id = sid
                break
        
        pcd_files = sorted(pointclouds_dir.glob("*.pcd"))
        logger.info(f"找到 {len(pcd_files)} 个点云文件")
        
        clouds_data = []
        for pcd_file in pcd_files:
            # 解析文件名：0_1763951779.000.pcd
            parts = pcd_file.stem.split("_")
            frame_id = int(parts[0])
            timestamp_sec = int(float(parts[1]))
            timestamp_nsec = int((float(parts[1]) - timestamp_sec) * 1e9)
            
            # 获取关联的pose_id
            self.cursor.execute(
                """
                SELECT pose_id FROM vehicle_poses 
                WHERE session_id = %s AND frame_id = %s
                """,
                (session_id, frame_id)
            )
            result = self.cursor.fetchone()
            pose_id = result[0] if result else None
            
            relative_path = str(pcd_file.relative_to(session_path))
            file_size = pcd_file.stat().st_size if pcd_file.exists() else 0
            
            clouds_data.append((
                session_id, pose_id, lidar_sensor_id,
                frame_id, timestamp_sec, timestamp_nsec,
                pcd_file.name, relative_path, str(pcd_file),
                'pcd', file_size
            ))
            
            if len(clouds_data) >= batch_size:
                self._batch_insert_clouds(clouds_data)
                clouds_data = []
        
        if clouds_data:
            self._batch_insert_clouds(clouds_data)
        
        logger.info("导入点云索引完成")
    
    def _batch_insert_clouds(self, clouds_data: List[Tuple]):
        """批量插入点云数据"""
        query = """
            INSERT INTO point_clouds
            (session_id, pose_id, sensor_id, frame_id, timestamp_sec, timestamp_nsec,
             cloud_name, relative_path, absolute_path, format, file_size_bytes)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            ON CONFLICT (session_id, sensor_id, frame_id) DO NOTHING
        """
        ok = self._safe_execute_batch(query, clouds_data, desc="point_clouds")
        if not ok:
            logger.error("部分点云批量插入失败，已跳过该批次")

    def _safe_execute_batch(self, query: str, params: List[Tuple], desc: str = None) -> bool:
        """在 savepoint 下执行批量插入，失败时回滚到 savepoint 并返回 False

        返回 True 表示批量执行成功并已提交，False 表示已回滚并跳过该批次。
        """
        # create a unique savepoint name
        self._sp_counter += 1
        sp_name = f"sp_{self._sp_counter}"
        try:
            self.cursor.execute(f"SAVEPOINT {sp_name}")
            execute_batch(self.cursor, query, params)
            # release savepoint is optional; commit to persist
            try:
                self.cursor.execute(f"RELEASE SAVEPOINT {sp_name}")
            except Exception:
                # some PG versions don't need explicit release
                pass
            self.conn.commit()
            return True
        except Exception as e:
            logger.error(f"批量插入({desc})出错: {e}")
            try:
                # rollback to savepoint to clear partial changes
                self.cursor.execute(f"ROLLBACK TO SAVEPOINT {sp_name}")
                self.conn.commit()
            except Exception as e2:
                logger.error(f"回滚到 savepoint 失败: {e2}")
                try:
                    self.conn.rollback()
                except Exception:
                    pass
            return False

    
    def import_submaps(self, session_path: Path, session_id: int):
        """导入子地图数据"""
        sparse_dir = session_path / "sparse"
        if not sparse_dir.exists():
            logger.warning(f"稀疏目录不存在: {sparse_dir}")
            return
        
        # 检查该session的子地图数据是否已存在
        self.cursor.execute(
            "SELECT COUNT(*) FROM submaps WHERE session_id = %s",
            (session_id,)
        )
        existing_count = self.cursor.fetchone()[0]
        if existing_count > 0:
            logger.info(f"子地图数据已存在({existing_count}条)，跳过导入")
            return
        
        submap_files = sorted(sparse_dir.glob("submap_*.pcd"))
        logger.info(f"找到 {len(submap_files)} 个子地图文件")
        
        for submap_file in submap_files:
            # 解析文件名：submap_3_5.pcd
            parts = submap_file.stem.split("_")
            if len(parts) >= 3:
                submap_index = int(parts[1])
                submap_version = int(parts[2])
            else:
                continue
            
            relative_path = str(submap_file.relative_to(session_path))
            file_size = submap_file.stat().st_size if submap_file.exists() else 0
            
            # 检查是否已存在
            self.cursor.execute(
                """
                SELECT submap_id FROM submaps 
                WHERE session_id = %s AND submap_name = %s
                """,
                (session_id, submap_file.name)
            )
            
            if self.cursor.fetchone():
                continue
            
            self.cursor.execute(
                """
                INSERT INTO submaps
                (session_id, submap_name, submap_index, submap_version,
                 relative_path, absolute_path, format, file_size_bytes)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                """,
                (session_id, submap_file.name, submap_index, submap_version,
                 relative_path, str(submap_file), 'pcd', file_size)
            )
        
        self.conn.commit()
        logger.info("导入子地图完成")
    
    def import_labels(self, session_path: Path, session_id: int, batch_size: int = 1000):
        """
        导入labels标注数据到labels表
        
        Args:
            session_path: 会话路径
            session_id: 会话ID
            batch_size: 批量插入大小
        """
        labels_dir = session_path / "labels"
        if not labels_dir.exists():
            logger.info(f"labels目录不存在，跳过: {labels_dir}")
            return
        
        # 检查该session的labels数据是否已存在
        self.cursor.execute(
            "SELECT COUNT(*) FROM labels WHERE session_id = %s",
            (session_id,)
        )
        existing_count = self.cursor.fetchone()[0]
        if existing_count > 0:
            logger.info(f"labels数据已存在({existing_count}条)，跳过导入")
            return
        
        # 获取所有标注文件并排序
        label_files = sorted(labels_dir.glob("*.json"))
        logger.info(f"找到 {len(label_files)} 个label文件")
        
        if len(label_files) == 0:
            return
        
        labels_data = []
        for label_file in label_files:
            # 解析文件名：0_1763951779.000.json （与odoms格式相同）
            parts = label_file.stem.split("_")
            if len(parts) < 2:
                logger.warning(f"标注文件名格式错误: {label_file.name}")
                continue
                
            frame_id = int(parts[0])
            timestamp_sec = int(float(parts[1]))
            timestamp_nsec = int((float(parts[1]) - timestamp_sec) * 1e9)
            
            # 计算文件大小
            file_size = label_file.stat().st_size
            
            # 计算相对路径
            relative_path = f"labels/{label_file.name}"
            
            # 读取JSON数据（可选）
            label_data = None
            try:
                with open(label_file, 'r', encoding='utf-8') as f:
                    label_data = json.load(f)
            except Exception as e:
                logger.warning(f"无法读取标注文件内容 {label_file.name}: {e}")
            
            # 准备插入数据
            label_row = (
                session_id,
                frame_id,
                timestamp_sec,
                timestamp_nsec,
                label_file.name,
                relative_path,
                str(label_file),
                file_size,
                json.dumps(label_data) if label_data else None
            )
            labels_data.append(label_row)
            
            # 批量插入
            if len(labels_data) >= batch_size:
                self._batch_insert_labels(labels_data)
                labels_data = []
        
        # 插入剩余数据
        if labels_data:
            self._batch_insert_labels(labels_data)
        
        logger.info(f"导入labels数据完成: {len(label_files)} 条")
    
    def _batch_insert_labels(self, labels_data: List[Tuple]):
        """批量插入labels数据到labels表"""
        query = """
            INSERT INTO labels
            (session_id, frame_id, timestamp_sec, timestamp_nsec,
             label_name, relative_path, absolute_path, file_size_bytes, label_data)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
            ON CONFLICT (session_id, frame_id) DO NOTHING
        """
        ok = self._safe_execute_batch(query, labels_data, desc="labels")
        if not ok:
            logger.error("部分labels批量插入失败，已跳过该批次")
    
    def import_full_session(self, session_path: Path, project_name: str,
                           session_name: str = None):
        """
        导入完整的会话数据
        
        Args:
            session_path: 会话数据目录路径
            project_name: 项目名称
            session_name: 会话名称（可选）
        """
        session_path = Path(session_path)
        
        if not session_path.exists():
            logger.error(f"会话路径不存在: {session_path}")
            return
        
        logger.info(f"开始处理会话: {session_path}")
        
        # 1. 创建或获取项目
        project_id = self.create_or_get_project(project_name)
        
        # 2. 导入传感器
        calib_dir = session_path / "calib"
        sensor_mapping = self.import_sensors(calib_dir)
        
        # 3. 创建会话并检查是否为新会话
        session_id, is_new = self.import_session(session_path, project_id, session_name)
        
        # 无论会话是否已存在，都执行数据导入（支持增量更新）
        # 数据库的唯一约束（ON CONFLICT DO NOTHING）会自动避免重复插入
        if is_new:
            logger.info(f"导入新会话数据: {session_name or session_path.name}")
        else:
            logger.info(f"会话已存在，执行增量数据导入: {session_name or session_path.name}")
        
        # 4. 导入标定数据
        self.import_calibrations(session_path, session_id, sensor_mapping)
        
        # 5. 导入odom位姿数据（vehicle_poses表）
        self.import_odoms(session_path, session_id)
        
        # 6. 导入优化位姿数据（optimized_poses表）
        self.import_poses(session_path, session_id)
        
        # 7. 导入图像索引
        self.import_images(session_path, session_id, sensor_mapping)
        
        # 8. 导入点云索引
        self.import_point_clouds(session_path, session_id, sensor_mapping)
        
        # 9. 导入子地图
        self.import_submaps(session_path, session_id)
        
        # 10. 导入标注数据（labels）
        self.import_labels(session_path, session_id)
        
        logger.info(f"新会话导入完成: {session_path}")


def main():
    """主函数"""
    # 加载配置文件
    config = load_config()
    
    # 数据库配置
    db_config = {
        'database': config['database']['name'],
        'user': config['database']['user']
    }
    
    # 如果配置了 host 和 port，添加到配置中
    if config['database'].get('host'):
        db_config['host'] = config['database']['host']
    if config['database'].get('port'):
        db_config['port'] = config['database']['port']
    if config['database'].get('password'):
        db_config['password'] = config['database']['password']
    
    # 数据路径
    data_root = Path(config['data_import']['data_root'])
    project_name = config['data_import']['project_name']
    
    # 自动遍历data_root下的所有子目录
    if not data_root.exists():
        logger.error(f"数据根目录不存在: {data_root}")
        return
    
    # 获取所有子目录并排序
    session_dirs = sorted([d for d in data_root.iterdir() if d.is_dir()])
    
    if not session_dirs:
        logger.warning(f"在 {data_root} 下未找到任何会话目录")
        return
    
    logger.info(f"找到 {len(session_dirs)} 个会话目录:")
    for i, session_dir in enumerate(session_dirs, 1):
        logger.info(f"  {i}. {session_dir.name}")

    # 用于统计导入结果
    imported_sessions = []  # 新导入的会话
    skipped_sessions = []   # 跳过的已存在会话
    failed_sessions = []    # 导入失败的会话

    try:
        with MappingDataImporter(db_config) as importer:
            for i, session_dir in enumerate(session_dirs, 1):
                logger.info(f"\n{'='*60}")
                logger.info(f"处理会话 {i}/{len(session_dirs)}: {session_dir.name}")
                logger.info(f"{'='*60}")
                
                if session_dir.exists():
                    session_name = session_dir.name  # 使用目录名作为会话名
                    try:
                        # import_full_session 会根据会话是否存在返回相应结果
                        # 如果是新会话，会完整导入；如果已存在，会跳过导入
                        importer.import_full_session(session_dir, project_name, session_name)
                        
                        # 检查会话是否已存在来决定分类
                        # 通过日志消息判断（或者可以修改import_full_session返回值）
                        with importer.conn.cursor() as cur:
                            cur.execute("""
                                SELECT session_id, created_at 
                                FROM data_sessions 
                                WHERE session_path = %s
                            """, (str(session_dir),))
                            result = cur.fetchone()
                            if result:
                                session_id, created_at = result
                                # 如果是最近几秒内创建的，可能是本次导入的
                                from datetime import datetime, timedelta
                                if datetime.now() - created_at < timedelta(seconds=5):
                                    imported_sessions.append(session_name)
                                else:
                                    skipped_sessions.append(session_name)
                    except Exception as e:
                        logger.error(f"导入会话 {session_name} 时出错: {e}")
                        # if the connection is in aborted state, rollback to clear it
                        try:
                            if importer and getattr(importer, 'conn', None):
                                importer.conn.rollback()
                                logger.info("发生错误，已对数据库连接执行 rollback，以便继续处理下一个会话")
                        except Exception as rb_e:
                            logger.error(f"回滚连接失败: {rb_e}")
                        failed_sessions.append((session_name, str(e)))
                else:
                    logger.warning(f"会话目录不存在: {session_dir}")
                    failed_sessions.append((session_dir.name, "目录不存在"))
        
        # 打印汇总报告
        logger.info("\n" + "="*70)
        logger.info("数据导入汇总报告")
        logger.info("="*70)
        logger.info(f"总计处理: {len(session_dirs)} 个会话")
        logger.info(f"  ✓ 新导入: {len(imported_sessions)} 个")
        logger.info(f"  ⊙ 跳过（已存在）: {len(skipped_sessions)} 个")
        logger.info(f"  ✗ 失败: {len(failed_sessions)} 个")
        
        if imported_sessions:
            logger.info("\n新导入的会话:")
            for name in imported_sessions:
                logger.info(f"  + {name}")
        
        if skipped_sessions:
            logger.info("\n跳过的会话（已存在）:")
            for name in skipped_sessions:
                logger.info(f"  ○ {name}")
        
        if failed_sessions:
            logger.info("\n导入失败的会话:")
            for name, error in failed_sessions:
                logger.info(f"  × {name}: {error}")
        
        logger.info("="*70)
        
    except Exception as e:
        logger.error(f"导入过程出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
