#!/usr/bin/env python3
"""
简单的数据库状态查看脚本
"""

import psycopg2
from psycopg2.extras import RealDictCursor
import yaml
from pathlib import Path


def load_config(config_file: str = None) -> dict:
    """
    加载配置文件
    
    Args:
        config_file: 配置文件路径，默认为脚本所在目录的 config.yaml
        
    Returns:
        配置字典
    """
    if config_file is None:
        # 默认使用脚本所在目录的 config.yaml
        script_dir = Path(__file__).parent
        config_file = script_dir / "config.yaml"
    
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    return config


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


def print_header(text):
    """打印标题"""
    print("\n" + "="*70)
    print(f"  {text}")
    print("="*70)


def print_table(cursor, query, max_rows=20):
    """执行查询并打印表格"""
    cursor.execute(query)
    results = cursor.fetchall()
    
    if not results:
        print("  (无数据)")
        return
    
    # 获取列名和宽度
    col_names = [desc[0] for desc in cursor.description]
    col_widths = [len(name) for name in col_names]
    
    # 计算每列的最大宽度
    for row in results[:max_rows]:
        for i, val in enumerate(row.values() if isinstance(row, dict) else row):
            col_widths[i] = max(col_widths[i], len(str(val)))
    
    # 限制列宽
    col_widths = [min(w, 50) for w in col_widths]
    
    # 打印表头
    header = " | ".join(name.ljust(width) for name, width in zip(col_names, col_widths))
    print(f"  {header}")
    print(f"  {'-' * len(header)}")
    
    # 打印数据行
    for row in results[:max_rows]:
        values = row.values() if isinstance(row, dict) else row
        line = " | ".join(str(v)[:width].ljust(width) for v, width in zip(values, col_widths))
        print(f"  {line}")
    
    if len(results) > max_rows:
        print(f"  ... (还有 {len(results) - max_rows} 行)")
    
    print(f"\n  总计: {len(results)} 行")


def main():
    try:
        conn = psycopg2.connect(**db_config, cursor_factory=RealDictCursor)
        cursor = conn.cursor()
        
        print("\n" + "█"*70)
        print("█" + " "*68 + "█")
        print("█" + "移动测绘系统数据库状态".center(57) + "█")
        print("█" + " "*68 + "█")
        print("█"*70)
        
        # 数据库概览
        print_header("📊 数据库概览")
        print_table(cursor, """
            SELECT 
                (SELECT COUNT(*) FROM projects) as "项目数",
                (SELECT COUNT(*) FROM data_sessions) as "会话数",
                (SELECT COUNT(*) FROM sensors) as "传感器数",
                (SELECT COUNT(*) FROM vehicle_poses) as "位姿数",
                (SELECT COUNT(*) FROM images) as "图像数",
                (SELECT COUNT(*) FROM point_clouds) as "点云数",
                (SELECT COUNT(*) FROM submaps) as "子地图数";
        """)
        
        # 项目列表
        print_header("📁 项目列表")
        print_table(cursor, """
            SELECT 
                project_id as "ID",
                project_name as "项目名称",
                location as "位置",
                TO_CHAR(created_at, 'YYYY-MM-DD HH24:MI:SS') as "创建时间",
                description as "描述"
            FROM projects
            ORDER BY project_id;
        """)
        
        # 会话列表
        print_header("🗂️  数据采集会话")
        print_table(cursor, """
            SELECT 
                ds.session_id as "ID",
                ds.session_name as "会话名称",
                ds.session_path as "会话路径",
                TO_CHAR(ds.start_time, 'MM-DD HH24:MI') as "开始时间",
                ROUND(ds.duration_seconds::numeric, 0) as "时长(秒)",
                ds.message_count as "消息数",
                COUNT(DISTINCT vp.pose_id) as "位姿数",
                COUNT(DISTINCT i.image_id) as "图像数"
            FROM data_sessions ds
            LEFT JOIN vehicle_poses vp ON ds.session_id = vp.session_id
            LEFT JOIN images i ON ds.session_id = i.session_id
            GROUP BY ds.session_id, ds.session_path
            ORDER BY ds.session_id;
        """)
        
        # 传感器列表
        print_header("📷 传感器列表")
        print_table(cursor, """
            SELECT 
                s.sensor_id as "ID",
                s.sensor_name as "名称",
                s.sensor_type as "类型",
                COUNT(DISTINCT i.image_id) as "图像数",
                COUNT(DISTINCT pc.cloud_id) as "点云数"
            FROM sensors s
            LEFT JOIN images i ON s.sensor_id = i.sensor_id
            LEFT JOIN point_clouds pc ON s.sensor_id = pc.sensor_id
            GROUP BY s.sensor_id
            ORDER BY s.sensor_name;
        """)
        
        # 数据分布
        print_header("📈 数据分布（按会话和传感器）")
        print_table(cursor, """
            SELECT 
                ds.session_name as "会话",
                s.sensor_name as "传感器",
                COUNT(*) as "数据量"
            FROM images i
            JOIN data_sessions ds ON i.session_id = ds.session_id
            JOIN sensors s ON i.sensor_id = s.sensor_id
            GROUP BY ds.session_id, ds.session_name, s.sensor_name
            ORDER BY ds.session_id, s.sensor_name;
        """, max_rows=15)
        
        # 存储空间
        print_header("💾 数据库存储空间")
        print_table(cursor, """
            SELECT 
                tablename as "表名",
                pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) AS "大小"
            FROM pg_tables
            WHERE schemaname = 'public'
            ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC
            LIMIT 15;
        """)
        
        # 文件路径示例
        print_header("📂 文件路径示例")
        print_table(cursor, """
            SELECT 
                'images' as "数据类型",
                s.sensor_name as "传感器",
                i.image_name as "文件名",
                LEFT(i.relative_path, 50) as "相对路径"
            FROM images i
            JOIN sensors s ON i.sensor_id = s.sensor_id
            ORDER BY i.image_id
            LIMIT 5;
        """)
        
        print_table(cursor, """
            SELECT 
                'point_clouds' as "数据类型",
                s.sensor_name as "传感器",
                pc.cloud_name as "文件名",
                LEFT(pc.relative_path, 50) as "相对路径"
            FROM point_clouds pc
            JOIN sensors s ON pc.sensor_id = s.sensor_id
            ORDER BY pc.cloud_id
            LIMIT 5;
        """)
        
        print_table(cursor, """
            SELECT 
                'submaps' as "数据类型",
                '-' as "传感器",
                sm.submap_name as "文件名",
                LEFT(sm.relative_path, 50) as "相对路径"
            FROM submaps sm
            ORDER BY sm.submap_id
            LIMIT 5;
        """)
        
        # 最近位姿
        print_header("🗺️  最近10个位姿")
        cursor.execute("SELECT MAX(session_id) FROM vehicle_poses;")
        latest_session = cursor.fetchone()['max']
        
        if latest_session:
            print_table(cursor, f"""
                SELECT 
                    frame_id as "帧ID",
                    ROUND(position_x::numeric, 2) as "X",
                    ROUND(position_y::numeric, 2) as "Y",
                    ROUND(position_z::numeric, 2) as "Z",
                    ROUND(latitude::numeric, 6) as "纬度",
                    ROUND(longitude::numeric, 6) as "经度",
                    ROUND(heading::numeric, 1) as "航向"
                FROM vehicle_poses
                WHERE session_id = {latest_session}
                ORDER BY frame_id DESC
                LIMIT 10;
            """)
        
        print("\n" + "█"*70)
        print("█" + " "*68 + "█")
        print("█" + "  查询完成！".center(65) + "█")
        print("█" + " "*68 + "█")
        print("█"*70 + "\n")
        
        cursor.close()
        conn.close()
        
    except psycopg2.OperationalError as e:
        print(f"\n❌ 数据库连接失败: {e}")
        print("\n请检查:")
        print("  1. PostgreSQL服务是否运行")
        print("  2. 数据库名称是否正确")
        print("  3. 用户名和密码是否正确")
        print("  4. 主机和端口是否正确")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
