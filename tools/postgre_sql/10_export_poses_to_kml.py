#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导出位姿数据为KML文件
从数据库的 vehicle_poses 表读取 GPS 坐标并生成 KML 文件，可在 Google Earth 中查看
"""

import os
import psycopg2
from psycopg2.extras import RealDictCursor
import yaml
from pathlib import Path
import argparse
import sys
import colorsys
from qt.load_config import load_config
# 导入3D box投影工具函数
from qt.box_projection import (
    lonlat_to_utm,
)

# 预定义的颜色方案（KML格式：AABBGGRR）- 基础色板
COLOR_PALETTE = [
    ('ff0000ff', '红色'),          # Red
    ('ff00ff00', '绿色'),          # Green
    ('ffff0000', '蓝色'),          # Blue
    ('ff00ffff', '黄色'),          # Yellow
    ('ffff00ff', '品红'),          # Magenta
    ('ffffff00', '青色'),          # Cyan
    ('ff0080ff', '橙色'),          # Orange
    ('ffff0080', '紫色'),          # Purple
    ('ff00ff80', '黄绿'),          # Yellow-Green
    ('ff80ff00', '天蓝'),          # Sky Blue
    ('ff8000ff', '深橙'),          # Dark Orange
    ('ffff8000', '深紫'),          # Dark Purple
    ('ff0040ff', '深红'),          # Dark Red
    ('ff00ff40', '亮绿'),          # Bright Green
    ('ffff4000', '深蓝'),          # Dark Blue
    ('ff40c0ff', '珊瑚红'),        # Coral
    ('ff40ffc0', '春绿'),          # Spring Green
    ('ffc0ff40', '天蓝紫'),        # Sky Blue Purple
    ('ffc04040', '深青'),          # Dark Cyan
    ('ff4040c0', '暗红'),          # Dark Red
    ('ff80c0ff', '浅橙'),          # Light Orange
    ('ff80ffc0', '浅绿'),          # Light Green
    ('ffc080ff', '淡紫'),          # Light Purple
    ('ffc0c080', '橄榄'),          # Olive
    ('ff8080c0', '灰紫'),          # Gray Purple
    ('ffc08080', '灰蓝'),          # Gray Blue
    ('ff80c080', '灰绿'),          # Gray Green
    ('ffff8080', '淡蓝紫'),        # Light Blue Purple
    ('ff80ff80', '淡黄绿'),        # Light Yellow Green
    ('ff8080ff', '淡红橙'),        # Light Red Orange
]


def generate_color_for_index(index: int) -> tuple:
    """
    根据索引生成独特的颜色
    
    使用HSV色彩空间生成均匀分布的颜色
    
    Args:
        index: 颜色索引
        
    Returns:
        (color_code, color_name) 元组
    """
    # 如果索引在预定义色板范围内，直接使用
    if index < len(COLOR_PALETTE):
        return COLOR_PALETTE[index]
    
    # 超出范围后，使用HSV色彩空间动态生成
    # 使用黄金角度（137.5°）确保颜色均匀分布
    golden_angle = 137.5
    hue = (index * golden_angle) % 360 / 360.0
    
    # 使用多层饱和度和亮度，增加颜色多样性
    saturation_levels = [1.0, 0.7, 0.85, 0.6]
    value_levels = [1.0, 0.8, 0.9, 0.7]
    
    sat_index = (index // 360) % len(saturation_levels)
    val_index = (index // (360 * len(saturation_levels))) % len(value_levels)
    
    saturation = saturation_levels[sat_index]
    value = value_levels[val_index]
    
    # 转换HSV到RGB
    r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
    
    # 转换为KML颜色格式（AABBGGRR）
    color_code = f'ff{int(b*255):02x}{int(g*255):02x}{int(r*255):02x}'
    color_name = f'自动色#{index+1}'
    
    return (color_code, color_name)


def save_poses_to_pcd(poses: list, output_file: str, sessions: dict = None):
    """
    将位姿数据保存为PCD文件
    
    Args:
        poses: 位姿数据列表
        output_file: 输出PCD文件路径
        sessions: 会话数据字典（可选，用于添加session_id字段）
    """
    print(f"\n正在保存PCD文件到: {output_file}")
    
    if not poses:
        print("⚠️ 警告: 没有位姿数据可保存")
        return
    
    # 将第一个点的经纬度转换为UTM坐标作为原点
    first_lon = poses[0]['longitude']
    first_lat = poses[0]['latitude']
    origin_x, origin_y, zone, hemisphere = lonlat_to_utm(first_lon, first_lat)
    origin_z = poses[0]['altitude'] if poses[0]['altitude'] is not None else 0
    
    print(f"  原点坐标 (经纬度): ({first_lon:.8f}, {first_lat:.8f}, {origin_z:.2f}m)")
    print(f"  原点坐标 (UTM): ({origin_x:.3f}, {origin_y:.3f}, {origin_z:.3f}m)")
    print(f"  UTM Zone: {zone}{hemisphere}")
    
    with open(output_file, 'w', encoding='utf-8') as f:
        # PCD文件头
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write(f'# Coordinate system: UTM (relative to first point)\n')
        f.write(f'# Origin (lon, lat, alt): {first_lon:.8f}, {first_lat:.8f}, {origin_z:.2f}\n')
        f.write(f'# Origin (UTM x, y, z): {origin_x:.2f}, {origin_y:.2f}, {origin_z:.2f}\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {len(poses)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(poses)}\n')
        f.write('DATA ascii\n')
        
        # 写入点数据（转换为UTM相对坐标）
        for pose in poses:
            lon = pose['longitude']
            lat = pose['latitude']
            alt = pose['altitude'] if pose['altitude'] is not None else 0
            
            # 转换经纬度到UTM
            utm_x, utm_y, _, _ = lonlat_to_utm(lon, lat)
            
            # 计算相对于原点的坐标
            rel_x = utm_x - origin_x
            rel_y = utm_y - origin_y
            rel_z = alt - origin_z
            
            f.write(f'{rel_x:.6f} {rel_y:.6f} {rel_z:.6f}\n')
    
    print(f"✓ PCD文件已保存: {output_file}")
    print(f"  点数: {len(poses)}")
    print(f"  坐标系: UTM相对坐标（以第一个点为原点）")


def export_poses_to_kml(db_config: dict, output_file: str, session_id: int = None, 
                        sample_rate: int = 5, color: str = None, save_pcd: str = None,
                        session_names: list = None):
    """
    从数据库导出位姿数据为KML文件
    
    Args:
        db_config: 数据库配置字典
        output_file: 输出KML文件路径
        session_id: 会话ID（可选，如果指定则只导出该会话的数据）
        sample_rate: 采样率（默认1表示导出所有点，10表示每隔10个点取1个）
        color: 轨迹颜色，AABBGGRR格式（可选，不指定则自动为每个会话分配颜色）
        save_pcd: PCD文件输出路径（可选，指定则同时保存PCD文件）
        session_names: 会话名称列表（可选，用于过滤特定会话）
    """
    print("="*80)
    print("导出位姿数据为KML文件")
    print("="*80)
    
    # 连接数据库
    try:
        conn = psycopg2.connect(**db_config, cursor_factory=RealDictCursor)
        cursor = conn.cursor()
        print("✓ 数据库连接成功")
    except Exception as e:
        print(f"✗ 数据库连接失败: {e}")
        sys.exit(1)
    
    try:
        # 构建查询SQL
        if session_id:
            query = """
                SELECT 
                    vp.pose_id,
                    vp.session_id,
                    vp.frame_id,
                    vp.longitude,
                    vp.latitude,
                    vp.altitude,
                    vp.timestamp_sec,
                    ds.session_name
                FROM vehicle_poses vp
                JOIN data_sessions ds ON vp.session_id = ds.session_id
                WHERE vp.session_id = %s
                    AND vp.longitude IS NOT NULL 
                    AND vp.latitude IS NOT NULL
                ORDER BY vp.timestamp_sec, vp.frame_id
            """
            cursor.execute(query, (session_id,))
            print(f"查询条件: session_id = {session_id}")
        else:
            # 根据是否有 session_names 过滤条件构建查询
            if session_names:
                query = """
                    SELECT 
                        vp.pose_id,
                        vp.session_id,
                        vp.frame_id,
                        vp.longitude,
                        vp.latitude,
                        vp.altitude,
                        vp.timestamp_sec,
                        ds.session_name
                    FROM vehicle_poses vp
                    JOIN data_sessions ds ON vp.session_id = ds.session_id
                    WHERE vp.longitude IS NOT NULL 
                        AND vp.latitude IS NOT NULL
                        AND ds.session_name = ANY(%s)
                    ORDER BY vp.session_id, vp.timestamp_sec, vp.frame_id
                """
                cursor.execute(query, (session_names,))
                print(f"查询条件: 指定会话 {session_names}")
            else:
                query = """
                    SELECT 
                        vp.pose_id,
                        vp.session_id,
                        vp.frame_id,
                        vp.longitude,
                        vp.latitude,
                        vp.altitude,
                        vp.timestamp_sec,
                        ds.session_name
                    FROM vehicle_poses vp
                    JOIN data_sessions ds ON vp.session_id = ds.session_id
                    WHERE vp.longitude IS NOT NULL 
                        AND vp.latitude IS NOT NULL
                    ORDER BY vp.session_id, vp.timestamp_sec, vp.frame_id
                """
                cursor.execute(query)
                print(f"查询条件: 所有会话")
        
        poses = cursor.fetchall()
        print(f"✓ 查询到 {len(poses)} 个有效位姿点")
        
        if not poses:
            print("警告: 没有找到有效的GPS坐标数据")
            return
        
        # 应用采样
        if sample_rate > 1:
            original_count = len(poses)
            poses = [poses[i] for i in range(0, len(poses), sample_rate)]
            print(f"✓ 采样后保留 {len(poses)} 个点 (采样率: 1/{sample_rate})")
        
        # 按会话分组
        sessions = {}
        for pose in poses:
            sid = pose['session_id']
            if sid not in sessions:
                sessions[sid] = {
                    'name': pose['session_name'],
                    'poses': []
                }
            sessions[sid]['poses'].append(pose)
        
        print(f"✓ 数据来自 {len(sessions)} 个会话")
        
        # 为每个会话分配颜色
        if color:
            # 如果指定了颜色，所有会话使用相同颜色
            session_colors = {sid: color for sid in sessions.keys()}
            print(f"✓ 使用指定颜色: {color}")
        else:
            # 自动为每个会话分配不同颜色
            session_colors = {}
            # print(f"✓ 自动分配颜色:")
            for idx, sid in enumerate(sorted(sessions.keys())):
                color_code, color_name = generate_color_for_index(idx)
                session_colors[sid] = color_code
                # print(f"  Session {sid:2d} ({sessions[sid]['name']:30s}): {color_name:15s} ({color_code})")
        
        # 生成KML文件
        write_kml_file(output_file, sessions, session_colors)
        
        print(f"✓ KML文件已保存: {output_file}")
        print(f"  可使用 Google Earth 打开查看")
        
        # 如果指定了保存PCD文件
        if save_pcd:
            save_poses_to_pcd(poses, save_pcd, sessions)
        
    except Exception as e:
        print(f"✗ 导出失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        cursor.close()
        conn.close()
    
    print("="*80)


def write_kml_file(filename: str, sessions: dict, session_colors: dict):
    """
    写入KML文件
    
    Args:
        filename: 输出文件路径
        sessions: 会话数据字典 {session_id: {'name': ..., 'poses': [...]}}
        session_colors: 会话颜色映射 {session_id: 'color_code'}
    """
    with open(filename, 'w', encoding='utf-8') as f:
        # KML头部
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('<Document>\n')
        f.write('  <name>Vehicle Poses Trajectory</name>\n')
        f.write('  <description>Exported from PostgreSQL/PostGIS database</description>\n')
        
        # 为每个会话定义独立的样式
        for session_id, color in session_colors.items():
            f.write(f'  <Style id="trajectoryStyle_{session_id}">\n')
            f.write('    <LineStyle>\n')
            f.write(f'      <color>{color}</color>\n')
            f.write('      <width>3</width>\n')
            f.write('    </LineStyle>\n')
            f.write('    <PolyStyle>\n')
            f.write(f'      <color>7f{color[2:]}</color>\n')  # 半透明版本
            f.write('    </PolyStyle>\n')
            f.write('  </Style>\n')
        
        # 定义起点和终点的样式
        f.write('  <Style id="startPointStyle">\n')
        f.write('    <IconStyle>\n')
        f.write('      <scale>0.8</scale>\n')
        f.write('      <color>ff00ff00</color>\n')  # 绿色起点
        f.write('      <Icon>\n')
        f.write('        <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n')
        f.write('      </Icon>\n')
        f.write('    </IconStyle>\n')
        f.write('  </Style>\n')
        
        f.write('  <Style id="endPointStyle">\n')
        f.write('    <IconStyle>\n')
        f.write('      <scale>0.8</scale>\n')
        f.write('      <color>ff0000ff</color>\n')  # 红色终点
        f.write('      <Icon>\n')
        f.write('        <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n')
        f.write('      </Icon>\n')
        f.write('    </IconStyle>\n')
        f.write('  </Style>\n')
        
        # 为每个会话创建一个Folder
        for session_id, session_data in sessions.items():
            session_name = session_data['name']
            poses = session_data['poses']
            color = session_colors[session_id]
            
            f.write(f'  <Folder>\n')
            f.write(f'    <name>Session: {session_name} (ID: {session_id})</name>\n')
            f.write(f'    <description>总点数: {len(poses)} | 颜色: {color}</description>\n')
            
            # 写入轨迹线
            f.write('    <Placemark>\n')
            f.write(f'      <name>轨迹线</name>\n')
            f.write(f'      <description>Session {session_id} trajectory</description>\n')
            f.write(f'      <styleUrl>#trajectoryStyle_{session_id}</styleUrl>\n')
            f.write('      <LineString>\n')
            f.write('        <tessellate>1</tessellate>\n')
            f.write('        <altitudeMode>absolute</altitudeMode>\n')
            f.write('        <coordinates>\n')
            
            for pose in poses:
                lon = pose['longitude']
                lat = pose['latitude']
                alt = pose['altitude'] if pose['altitude'] is not None else 0
                f.write(f'          {lon},{lat},{alt}\n')
            
            f.write('        </coordinates>\n')
            f.write('      </LineString>\n')
            f.write('    </Placemark>\n')
            
            # 为起点和终点添加标记
            if poses:
                # 起点
                start_pose = poses[0]
                f.write('    <Placemark>\n')
                # 起点标签：保留“起点”并显示该轨迹对应的 session 名称
                f.write(f'      <name>起点 - {session_name}</name>\n')
                f.write(f'      <description>起点<br/>Session: {session_name}<br/>Frame: {start_pose["frame_id"]}</description>\n')
                f.write('      <styleUrl>#startPointStyle</styleUrl>\n')
                f.write('      <Point>\n')
                f.write('        <altitudeMode>absolute</altitudeMode>\n')
                f.write(f'        <coordinates>{start_pose["longitude"]},{start_pose["latitude"]},{start_pose["altitude"] or 0}</coordinates>\n')
                f.write('      </Point>\n')
                f.write('    </Placemark>\n')
                
                # 终点
                end_pose = poses[-1]
                f.write('    <Placemark>\n')
                f.write('      <name>终点</name>\n')
                f.write(f'      <description>Session: {session_name}<br/>Frame: {end_pose["frame_id"]}</description>\n')
                f.write('      <styleUrl>#endPointStyle</styleUrl>\n')
                f.write('      <Point>\n')
                f.write('        <altitudeMode>absolute</altitudeMode>\n')
                f.write(f'        <coordinates>{end_pose["longitude"]},{end_pose["latitude"]},{end_pose["altitude"] or 0}</coordinates>\n')
                f.write('      </Point>\n')
                f.write('    </Placemark>\n')
            
            f.write('  </Folder>\n')
        
        # KML尾部
        f.write('</Document>\n')
        f.write('</kml>\n')


def main():
    """主函数"""
    # 加载配置文件
    config = load_config()
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
 

    # 命令行参数
    parser = argparse.ArgumentParser(
        description='导出位姿数据为KML文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 导出所有会话的位姿到KML
  python 10_export_poses_to_kml.py -o trajectory.kml
  
  # 导出指定会话
  python 10_export_poses_to_kml.py -o trajectory.kml -s 1
  
  # 使用采样（每10个点取1个）
  python 10_export_poses_to_kml.py -o trajectory.kml -r 10
  
  # 自定义颜色（红色: ff0000ff, 绿色: ff00ff00, 蓝色: ffff0000）
  python 10_export_poses_to_kml.py -o trajectory.kml --color ff00ff00
  
  # 同时导出KML和PCD文件（PCD使用UTM坐标，以第一个点为原点）
  python 10_export_poses_to_kml.py -o trajectory.kml -s 1 --save_pcd poses.pcd
  
  # 只导出PCD文件（-o参数仍然必需，但可以随便指定）
  python 10_export_poses_to_kml.py -o /tmp/dummy.kml -s 1 --save_pcd /data/poses.pcd
        """
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default='trajectory.kml',
        help='输出KML文件路径 (默认: trajectory.kml)'
    )
    
    parser.add_argument(
        '-s', '--session',
        type=int,
        default=None,
        help='会话ID（可选，不指定则导出所有会话）'
    )
    
    parser.add_argument(
        '-r', '--rate',
        type=int,
        default=1,
        help='采样率（默认: 1，表示所有点；10表示每10个点取1个）'
    )
    
    parser.add_argument(
        '--color',
        type=str,
        default=None,
        help='轨迹颜色，AABBGGRR格式（可选，不指定则自动为每个会话分配不同颜色）'
    )
    
    parser.add_argument(
        '--save_pcd',
        type=str,
        default=None,
        help='保存PCD文件路径（可选，指定则同时将位姿保存为PCD格式）'
    )

    args = parser.parse_args()
    merge_config = config.get('merge_session', {})
    output_config = merge_config.get('output', {})
    # 若输出为相对路径（且未包含目录），默认放到 config 里的 output_dir 下
    output_dir = output_config.get('output_dir', './output')
    if not os.path.isabs(args.output) and os.path.dirname(args.output) == "":
        os.makedirs(output_dir, exist_ok=True)
        args.output = os.path.join(output_dir, args.output)

    # 处理 save_pcd 路径
    save_pcd_path = None
    if args.save_pcd:
        if not os.path.isabs(args.save_pcd) and os.path.dirname(args.save_pcd) == "":
            save_pcd_path = os.path.join(output_dir, args.save_pcd)
        else:
            save_pcd_path = args.save_pcd
    
    # 获取会话过滤列表
    search_params = config.get('search_params', {})
    session_names = search_params.get('use_session_subdir', [])
    if session_names:
        print(f"限制导出会话: {', '.join(session_names)}")
    else:
        print("导出所有会话")
    
    # 执行导出
    export_poses_to_kml(
        db_config=db_config,
        output_file=args.output,
        session_id=args.session,
        sample_rate=args.rate,
        color=args.color,
        save_pcd=save_pcd_path,
        session_names=session_names if session_names else None
    )


if __name__ == "__main__":
    main()
