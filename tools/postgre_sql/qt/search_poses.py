#!/usr/bin/env python3
"""
位姿空间查询工具
根据给定的GPS坐标和距离范围，查询附近的所有位姿数据
"""

import psycopg2
from psycopg2.extras import RealDictCursor

import sys
from typing import List, Tuple, Optional

# 使用统一的配置加载方式
from qt.load_config import load_config

# 加载配置
config = load_config()

# 数据库配置
db_config = {
    'database': config['database']['name'],
    'user': config['database']['user']
}

if config['database'].get('host'):
    db_config['host'] = config['database']['host']
if config['database'].get('port'):
    db_config['port'] = config['database']['port']
if config['database'].get('password'):
    db_config['password'] = config['database']['password']


class PoseSearcher:
    """位姿搜索器"""
    
    def __init__(self, db_config):
        self.db_config = db_config
        self.conn = None
        self.cursor = None
    
    def connect(self):
        """连接数据库"""
        self.conn = psycopg2.connect(**self.db_config, cursor_factory=RealDictCursor)
        self.cursor = self.conn.cursor()
    
    def close(self):
        """关闭连接"""
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
    
    def search_poses_by_distance(self, 
                                 longitude: float, 
                                 latitude: float, 
                                 distance_meters: float,
                                 session_id: Optional[int] = None,
                                 session_names: Optional[List[str]] = None,
                                 limit: Optional[int] = None,
                                 utm_zone: int = 51) -> List[dict]:
        """
        根据距离搜索位姿（使用UTM坐标系）
        
        算法流程:
        1. 将给定的GPS经纬度投影到UTM坐标系
        2. 在optimized_poses表中查找UTM距离范围内的位姿
        3. JOIN其他表返回完整信息
        
        Args:
            longitude: 目标经度
            latitude: 目标纬度
            distance_meters: 搜索半径（米）
            session_id: 会话ID（可选，如果指定则只搜索该会话）
            session_names: 会话名称列表（可选，如果指定则只搜索这些会话）
            limit: 返回结果数量限制（可选）
            utm_zone: UTM区号（默认51，适用于中国东部地区）
            
        Returns:
            位姿列表
        """
        # 使用PostGIS将GPS坐标转换为UTM坐标
        # EPSG:326XX 表示北半球UTM，其中XX是区号
        utm_srid = 32600 + utm_zone
        
        # 添加会话过滤（需要添加到WHERE子句中）
        session_filter = ""
        params: List[object] = [
            longitude, latitude, utm_srid,  # target_utm: 转换GPS到UTM
            longitude, latitude, utm_srid,  # target_utm: 转换GPS到UTM (重复用于Y坐标)
            distance_meters, distance_meters,  # X轴范围
            distance_meters, distance_meters,  # Y轴范围
        ]

        if session_id is not None:
            session_filter += " AND op.session_id = %s"
            params.append(session_id)

        if session_names is not None and len(session_names) > 0:
            placeholders = ','.join(['%s'] * len(session_names))
            session_filter += f" AND ds.session_name IN ({placeholders})"
            params.extend(session_names)

        query = """
            WITH target_utm AS (
                -- 步骤1: 将目标GPS坐标投影到UTM坐标系
                SELECT 
                    ST_X(ST_Transform(ST_SetSRID(ST_MakePoint(%s, %s), 4326), %s)) as utm_x,
                    ST_Y(ST_Transform(ST_SetSRID(ST_MakePoint(%s, %s), 4326), %s)) as utm_y
            ),
            pose_data AS (
                SELECT 
                    op.opt_pose_id,
                    op.session_id,
                    op.frame_id,
                    op.timestamp_sec,
                    op.timestamp_nsec,
                    timestamp_from_unix(op.timestamp_sec, op.timestamp_nsec) as timestamp,
                    
                    -- UTM坐标
                    op.position_utm_x,
                    op.position_utm_y,
                    op.position_utm_z,
                    
                    -- GPS坐标
                    op.longitude,
                    op.latitude,
                    op.altitude,
                    
                    -- 计算UTM平面距离（欧式距离）
                    SQRT(
                        POWER(op.position_utm_x - target_utm.utm_x, 2) + 
                        POWER(op.position_utm_y - target_utm.utm_y, 2)
                    ) as distance_meters,
                    
                    -- 关联vehicle_poses获取额外信息
                    vp.pose_id,
                    vp.position_x,
                    vp.position_y,
                    vp.position_z,
                    vp.heading,
                    vp.velocity_x,
                    vp.velocity_y,
                    vp.velocity_z,
                    
                    -- 关联点云信息
                    pc.absolute_path as pointcloud_path,
                    pc.cloud_name as pointcloud_name,
                    
                    -- 会话信息
                    ds.session_path,
                    
                    -- 优化位姿时间戳信息
                    ROUND((op.timestamp_sec + op.timestamp_nsec::float / 1e9)::numeric, 3) as opt_timestamp_full
                    
                FROM optimized_poses op
                CROSS JOIN target_utm
                LEFT JOIN vehicle_poses vp ON op.session_id = vp.session_id AND op.frame_id = vp.frame_id
            LEFT JOIN point_clouds pc ON vp.pose_id = pc.pose_id
            LEFT JOIN data_sessions ds ON op.session_id = ds.session_id
                WHERE 
                    -- 步骤2: 使用UTM坐标进行空间过滤（矩形范围预筛选，利用索引）
                    op.position_utm_x BETWEEN (SELECT utm_x FROM target_utm) - %s 
                                          AND (SELECT utm_x FROM target_utm) + %s
                    AND op.position_utm_y BETWEEN (SELECT utm_y FROM target_utm) - %s 
                                              AND (SELECT utm_y FROM target_utm) + %s
                    __SESSION_FILTER__
            )
            SELECT 
                pd.*,
                -- 聚合图像路径（按传感器分组）
                COALESCE(
                    json_object_agg(
                        s.sensor_name, 
                        i.absolute_path
                    ) FILTER (WHERE i.image_id IS NOT NULL),
                    '{}'::json
                ) as image_paths,
                -- 图像数量
                COUNT(DISTINCT i.image_id) as image_count,
                -- odom位姿文件路径
                CASE 
                    WHEN pd.session_path IS NOT NULL THEN
                        pd.session_path || '/odoms/' || pd.frame_id || '_' || 
                        ROUND((pd.timestamp_sec + pd.timestamp_nsec::float / 1e9)::numeric, 3)::text || '.yaml'
                    ELSE NULL
                END as odom_path
            FROM pose_data pd
            LEFT JOIN images i ON pd.pose_id = i.pose_id
            LEFT JOIN sensors s ON i.sensor_id = s.sensor_id
            GROUP BY 
                pd.opt_pose_id, pd.session_id, pd.frame_id, pd.timestamp_sec, pd.timestamp_nsec,
                pd.timestamp, pd.position_utm_x, pd.position_utm_y, pd.position_utm_z,
                pd.longitude, pd.latitude, pd.altitude, pd.distance_meters,
                pd.pose_id, pd.position_x, pd.position_y, pd.position_z, pd.heading,
                pd.velocity_x, pd.velocity_y, pd.velocity_z,
                pd.pointcloud_path, pd.pointcloud_name, pd.session_path, pd.opt_timestamp_full
        """

        # 注入会话过滤（不使用 f-string，避免与 SQL 内的 '{}' 花括号冲突）
        query = query.replace("__SESSION_FILTER__", session_filter)
        
        # 排序和限制
        query += " ORDER BY pd.distance_meters"
        
        if limit is not None:
            query += " LIMIT %s"
            params.append(limit)
        
        query += ";"
        
        self.cursor.execute(query, params)
        return self.cursor.fetchall()
    
    def search_poses_by_bbox(self,
                            min_lon: float,
                            min_lat: float,
                            max_lon: float,
                            max_lat: float,
                            session_id: Optional[int] = None) -> List[dict]:
        """
        根据边界框搜索位姿
        
        Args:
            min_lon: 最小经度
            min_lat: 最小纬度
            max_lon: 最大经度
            max_lat: 最大纬度
            session_id: 会话ID（可选）
            
        Returns:
            位姿列表
        """
        query = """
            SELECT 
                vp.pose_id,
                vp.session_id,
                vp.frame_id,
                vp.timestamp_sec,
                timestamp_from_unix(vp.timestamp_sec, vp.timestamp_nsec) as timestamp,
                vp.position_x,
                vp.position_y,
                vp.position_z,
                vp.longitude,
                vp.latitude,
                vp.altitude,
                vp.heading
            FROM vehicle_poses vp
            WHERE vp.gps_position IS NOT NULL
                AND vp.gps_position && ST_MakeEnvelope(%s, %s, %s, %s, 4326)
        """
        
        params = [min_lon, min_lat, max_lon, max_lat]
        
        if session_id is not None:
            query += " AND vp.session_id = %s"
            params.append(session_id)
        
        query += " ORDER BY vp.frame_id;"
        
        self.cursor.execute(query, params)
        return self.cursor.fetchall()
    
    def get_nearest_pose(self, longitude: float, latitude: float, 
                        session_id: Optional[int] = None,
                        utm_zone: int = 51) -> Optional[dict]:
        """
        获取最近的位姿
        
        Args:
            longitude: 目标经度
            latitude: 目标纬度
            session_id: 会话ID（可选）
            utm_zone: UTM区号（默认51）
            
        Returns:
            最近的位姿（如果存在）
        """
        results = self.search_poses_by_distance(
            longitude, latitude, 
            distance_meters=1000,  # 搜索1km范围
            session_id=session_id,
            utm_zone=utm_zone,
            limit=1
        )
        return results[0] if results else None
    
    def get_poses_with_images(self,
                             longitude: float,
                             latitude: float,
                             distance_meters: float,
                             session_id: Optional[int] = None) -> List[dict]:
        """
        搜索位姿及其关联的图像
        
        Args:
            longitude: 目标经度
            latitude: 目标纬度
            distance_meters: 搜索半径（米）
            session_id: 会话ID（可选）
            
        Returns:
            位姿和图像信息
        """
        query = """
            SELECT 
                vp.pose_id,
                vp.frame_id,
                vp.longitude,
                vp.latitude,
                ST_Distance(
                    vp.gps_position::geography,
                    ST_SetSRID(ST_MakePoint(%s, %s), 4326)::geography
                ) as distance_meters,
                COUNT(i.image_id) as image_count,
                STRING_AGG(DISTINCT s.sensor_name, ', ') as sensors,
                ARRAY_AGG(i.absolute_path) as image_paths
            FROM vehicle_poses vp
            LEFT JOIN images i ON vp.pose_id = i.pose_id
            LEFT JOIN sensors s ON i.sensor_id = s.sensor_id
            WHERE vp.gps_position IS NOT NULL
                AND ST_DWithin(
                    vp.gps_position::geography,
                    ST_SetSRID(ST_MakePoint(%s, %s), 4326)::geography,
                    %s
                )
        """
        
        params = [longitude, latitude, longitude, latitude, distance_meters]
        
        if session_id is not None:
            query += " AND vp.session_id = %s"
            params.append(session_id)
        
        query += """
            GROUP BY vp.pose_id, vp.frame_id, vp.longitude, vp.latitude, vp.gps_position
            ORDER BY distance_meters
        """
        
        self.cursor.execute(query, params)
        return self.cursor.fetchall()


def print_poses(poses: List[dict], show_details: bool = False):
    """打印位姿信息"""
    if not poses:
        print("  未找到符合条件的位姿")
        return
    
    print(f"\n  找到 {len(poses)} 个位姿")
    print(f"  {'='*80}")
    
    if show_details:
        # 详细模式
        for i, pose in enumerate(poses, 1):
            print(f"\n  位姿 #{i}:")
            print(f"    优化位姿ID: {pose.get('opt_pose_id', 'N/A')}")
            print(f"    Frame ID: {pose['frame_id']}")
            print(f"    Session ID: {pose.get('session_id', 'N/A')}")
            print(f"    时间戳: {pose.get('timestamp', 'N/A')}")
            
            # UTM坐标（主要坐标）
            print(f"    UTM坐标: ({pose.get('position_utm_x', 'N/A'):.2f}, "
                  f"{pose.get('position_utm_y', 'N/A'):.2f}, {pose.get('position_utm_z', 'N/A'):.2f})")
            
            # GPS坐标
            if pose.get('latitude') and pose.get('longitude'):
                print(f"    GPS坐标: ({pose['latitude']:.6f}, {pose['longitude']:.6f}, {pose.get('altitude', 'N/A')})")
            
            # 局部坐标（来自vehicle_poses）
            if pose.get('position_x') is not None:
                print(f"    局部坐标: ({pose.get('position_x', 'N/A'):.2f}, {pose.get('position_y', 'N/A'):.2f}, {pose.get('position_z', 'N/A'):.2f})")
            
            if 'distance_meters' in pose:
                print(f"    距离: {pose['distance_meters']:.2f} 米")
            if 'heading' in pose and pose['heading'] is not None:
                print(f"    航向: {pose['heading']:.2f}°")
            if 'velocity_x' in pose and pose['velocity_x'] is not None:
                speed = (pose.get('velocity_x', 0)**2 + pose.get('velocity_y', 0)**2 + pose.get('velocity_z', 0)**2)**0.5
                print(f"    速度: {speed:.2f} m/s")
            
            # 显示点云路径
            if pose.get('pointcloud_path'):
                print(f"    点云文件: {pose['pointcloud_name']}")
                print(f"    点云路径: {pose['pointcloud_path']}")
            else:
                print(f"    点云文件: 无")
            
            # 显示odom位姿文件路径
            if pose.get('odom_path'):
                print(f"    Odom位姿路径: {pose['odom_path']}")
            
            # 显示优化位姿文件路径
            if pose.get('session_path') and pose.get('opt_timestamp_full'):
                # 使用完整时间戳（包含小数部分）
                opt_pose_path = f"{pose['session_path']}/sparse/vehicle_geo_pose/{pose['frame_id']}_{pose['opt_timestamp_full']:.3f}.yaml"
                print(f"    优化位姿路径: {opt_pose_path}")
            
            # 显示图像路径
            if pose.get('image_count', 0) > 0:
                print(f"    图像数量: {pose['image_count']}")
                if pose.get('image_paths'):
                    import json
                    image_paths_dict = pose['image_paths'] if isinstance(pose['image_paths'], dict) else json.loads(pose['image_paths'])
                    print(f"    图像路径:")
                    for sensor_name, img_path in image_paths_dict.items():
                        print(f"      {sensor_name}: {img_path}")
            else:
                print(f"    图像: 无")
            
            # 显示vehicle_poses关联信息
            if pose.get('pose_id'):
                print(f"    关联Vehicle Pose ID: {pose['pose_id']}")
    else:
        # 表格模式
        print(f"  {'OptID':<8} {'Frame':<8} {'UTM-X':<12} {'UTM-Y':<12} {'UTM-Z':<8} {'距离(m)':<10}")
        print(f"  {'-'*80}")
        for pose in poses:
            print(f"  {pose.get('opt_pose_id', 'N/A'):<8} {pose['frame_id']:<8} "
                  f"{pose.get('position_utm_x', 0):<12.2f} {pose.get('position_utm_y', 0):<12.2f} "
                  f"{pose.get('position_utm_z', 0):<8.2f} "
                  f"{pose.get('distance_meters', 0):<10.2f}")
        
        # 在表格模式下也显示路径摘要
        print(f"\n  {'='*80}")
        print(f"  文件路径信息:")
        for i, pose in enumerate(poses, 1):
            print(f"\n  [{i}] Frame {pose['frame_id']} (OptID: {pose.get('opt_pose_id', 'N/A')}):")
            
            # odom路径
            if pose.get('odom_path'):
                print(f"      Odom: {pose['odom_path']}")
            
            # 点云路径
            if pose.get('pointcloud_path'):
                print(f"      点云: {pose['pointcloud_path']}")
            
            # 优化位姿路径
            if pose.get('session_path') and pose.get('opt_timestamp_full'):
                opt_pose_path = f"{pose['session_path']}/sparse/vehicle_geo_pose/{pose['frame_id']}_{pose['opt_timestamp_full']:.3f}.yaml"
                print(f"      优化位姿: {opt_pose_path}")
            
            # 图像路径
            if pose.get('image_count', 0) > 0:
                import json
                image_paths_dict = pose['image_paths'] if isinstance(pose['image_paths'], dict) else json.loads(pose['image_paths'])
                print(f"      图像 ({pose['image_count']}张):")
                for sensor_name, img_path in image_paths_dict.items():
                    print(f"        {sensor_name}: {img_path}")


def main():
    """主函数"""
    print("\n" + "="*80)
    print("  位姿空间查询工具 (基于UTM坐标系)")
    print("="*80)
    
    # 示例查询
    with PoseSearcher(db_config) as searcher:
        # 示例1: 查询某个位置附近100米内的位姿
        print("\n示例1: 查询指定位置附近50米内的位姿 (使用UTM坐标系)")
        print("-" * 80)
 
        target_lon = 120.65582103  # 目标经度
        target_lat = 31.41033324   # 目标纬度
        distance = 100.0        # 搜索半径（米）
        utm_zone = int((target_lon + 180.0) / 6.0 + 1)  # UTM区号（中国东部地区使用51N）
        
        print(f"  目标位置: ({target_lat:.6f}, {target_lon:.6f})")
        print(f"  搜索半径: {distance} 米")
        print(f"  UTM区号: {utm_zone}N (EPSG:326{utm_zone:02d})")
        print(f"\n  算法流程:")
        print(f"    1. 将GPS坐标投影到UTM坐标系")
        print(f"    2. 在optimized_poses表中查找UTM距离范围内的位姿")
        print(f"    3. 关联vehicle_poses、point_clouds等表获取完整信息")
        
        poses = searcher.search_poses_by_distance(
            longitude=target_lon,
            latitude=target_lat,
            distance_meters=distance,
            utm_zone=utm_zone,
            limit=10000000
        )
        print_poses(poses, show_details=True)
        return
        
        # 示例2: 查询最近的位姿
        print("\n\n示例2: 查询最近的位姿（详细信息）")
        print("-" * 80)
        nearest = searcher.get_nearest_pose(target_lon, target_lat)
        if nearest:
            print_poses([nearest], show_details=True)
        
        # 示例3: 边界框查询
        print("\n\n示例3: 边界框查询")
        print("-" * 80)
        min_lon, min_lat = 120.655, 31.409
        max_lon, max_lat = 120.657, 31.411
        print(f"  边界框: ({min_lat:.6f}, {min_lon:.6f}) 到 ({max_lat:.6f}, {max_lon:.6f})")
        
        poses = searcher.search_poses_by_bbox(
            min_lon=min_lon, min_lat=min_lat,
            max_lon=max_lon, max_lat=max_lat
        )
        print_poses(poses[:10])  # 只显示前10个
        
        # 示例4: 查询带图像的位姿
        print("\n\n示例4: 查询附近有图像的位姿")
        print("-" * 80)
        poses_with_images = searcher.get_poses_with_images(
            longitude=target_lon,
            latitude=target_lat,
            distance_meters=500
        )
        if poses_with_images:
            print(f"\n  找到 {len(poses_with_images)} 个带图像的位姿")
            print(f"  {'='*80}")
            for pose in poses_with_images[:5]:  # 显示前5个
                print(f"\n  Frame {pose['frame_id']}:")
                print(f"    GPS: ({pose['latitude']:.6f}, {pose['longitude']:.6f})")
                print(f"    距离: {pose['distance_meters']:.2f} 米")
                print(f"    图像数量: {pose['image_count']}")
                if pose['sensors']:
                    print(f"    传感器: {pose['sensors']}")
    
    print("\n" + "="*80)
    print("  查询完成")
    print("="*80 + "\n")


def interactive_mode():
    """交互式查询模式"""
    print("\n" + "="*80)
    print("  位姿空间查询工具 - 交互模式")
    print("="*80)
    
    with PoseSearcher(db_config) as searcher:
        while True:
            print("\n请选择查询方式:")
            print("  1. 按距离查询")
            print("  2. 按边界框查询")
            print("  3. 查找最近的位姿")
            print("  4. 查询带图像的位姿")
            print("  0. 退出")
            
            choice = input("\n请输入选项 (0-4): ").strip()
            
            if choice == '0':
                print("退出程序")
                break
            
            elif choice == '1':
                try:
                    print("\n=== 按距离查询 (基于UTM坐标系) ===")
                    lat = float(input("请输入目标纬度: "))
                    lon = float(input("请输入目标经度: "))
                    distance = float(input("请输入搜索半径(米): "))
                    utm_zone_input = input("请输入UTM区号(默认51): ").strip()
                    utm_zone = int(utm_zone_input) if utm_zone_input else 51
                    limit = input("限制结果数量(回车跳过): ").strip()
                    limit = int(limit) if limit else None
                    
                    print(f"\n正在搜索 ({lat:.6f}, {lon:.6f}) 附近 {distance} 米内的位姿...")
                    print(f"使用UTM区号: {utm_zone}N")
                    poses = searcher.search_poses_by_distance(lon, lat, distance, 
                                                             utm_zone=utm_zone, limit=limit)
                    print_poses(poses, show_details=True)
                    
                except ValueError as e:
                    print(f"输入错误: {e}")
            
            elif choice == '2':
                try:
                    print("\n=== 按边界框查询 ===")
                    min_lat = float(input("请输入最小纬度: "))
                    min_lon = float(input("请输入最小经度: "))
                    max_lat = float(input("请输入最大纬度: "))
                    max_lon = float(input("请输入最大经度: "))
                    
                    print(f"\n正在搜索边界框内的位姿...")
                    poses = searcher.search_poses_by_bbox(min_lon, min_lat, max_lon, max_lat)
                    print_poses(poses, show_details=True)
                    
                except ValueError as e:
                    print(f"输入错误: {e}")
            
            elif choice == '3':
                try:
                    print("\n=== 查找最近的位姿 (基于UTM坐标系) ===")
                    lat = float(input("请输入目标纬度: "))
                    lon = float(input("请输入目标经度: "))
                    utm_zone_input = input("请输入UTM区号(默认51): ").strip()
                    utm_zone = int(utm_zone_input) if utm_zone_input else 51
                    
                    print(f"\n正在搜索最近的位姿...")
                    print(f"使用UTM区号: {utm_zone}N")
                    nearest = searcher.get_nearest_pose(lon, lat, utm_zone=utm_zone)
                    if nearest:
                        print_poses([nearest], show_details=True)
                    else:
                        print("未找到位姿")
                    
                except ValueError as e:
                    print(f"输入错误: {e}")
            
            elif choice == '4':
                try:
                    print("\n=== 查询带图像的位姿 ===")
                    lat = float(input("请输入目标纬度: "))
                    lon = float(input("请输入目标经度: "))
                    distance = float(input("请输入搜索半径(米): "))
                    
                    print(f"\n正在搜索...")
                    poses = searcher.get_poses_with_images(lon, lat, distance)
                    if poses:
                        print(f"\n找到 {len(poses)} 个带图像的位姿")
                        for i, pose in enumerate(poses[:10], 1):
                            print(f"\n  位姿 {i}:")
                            print(f"    Frame: {pose['frame_id']}")
                            print(f"    GPS: ({pose['latitude']:.6f}, {pose['longitude']:.6f})")
                            print(f"    距离: {pose['distance_meters']:.2f} 米")
                            print(f"    图像数: {pose['image_count']}")
                            if pose['sensors']:
                                print(f"    传感器: {pose['sensors']}")
                    else:
                        print("未找到位姿")
                    
                except ValueError as e:
                    print(f"输入错误: {e}")
            else:
                print("无效选项，请重新输入")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == '-i':
        # 交互模式
        interactive_mode()
    else:
        # 示例模式
        main()
        # print("\n提示: 使用 'python search_poses.py -i' 进入交互模式")
