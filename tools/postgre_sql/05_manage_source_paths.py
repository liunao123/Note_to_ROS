#!/usr/bin/env python3
"""
数据来源路径管理工具
用于设置和查询原始数据的存放路径
"""

import psycopg2
from psycopg2.extras import RealDictCursor
import argparse
import yaml
from pathlib import Path
from typing import Optional
import sys


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


class SourcePathManager:
    """数据来源路径管理器"""
    
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
        if exc_type is None:
            self.conn.commit()
        else:
            self.conn.rollback()
        self.close()
    
    def update_project_source_root(self, project_id: int, source_root: str):
        """更新项目的原始数据根目录"""
        self.cursor.execute(
            "UPDATE projects SET source_data_root = %s WHERE project_id = %s",
            (source_root, project_id)
        )
        self.conn.commit()
        print(f"✓ 已更新项目 {project_id} 的原始数据根目录: {source_root}")
    
    def update_session_source_path(self, session_id: int, source_path: str):
        """更新会话的原始数据路径"""
        self.cursor.execute(
            "UPDATE data_sessions SET source_data_path = %s WHERE session_id = %s",
            (source_path, session_id)
        )
        self.conn.commit()
        print(f"✓ 已更新会话 {session_id} 的原始数据路径: {source_path}")
    
    def update_session_source_path_by_name(self, session_name: str, source_path: str):
        """通过会话名称更新原始数据路径"""
        self.cursor.execute(
            "UPDATE data_sessions SET source_data_path = %s WHERE session_name = %s RETURNING session_id",
            (source_path, session_name)
        )
        result = self.cursor.fetchone()
        if result:
            self.conn.commit()
            print(f"✓ 已更新会话 '{session_name}' (ID: {result['session_id']}) 的原始数据路径: {source_path}")
        else:
            print(f"✗ 未找到会话名称: {session_name}")
    
    def batch_update_sessions_by_pattern(self, project_id: int, source_root: str, pattern: str = "{session_name}"):
        """
        批量更新会话的原始数据路径
        
        Args:
            project_id: 项目ID
            source_root: 原始数据根目录
            pattern: 路径模式，可使用 {session_name} 占位符
        """
        self.cursor.execute(
            "SELECT session_id, session_name FROM data_sessions WHERE project_id = %s",
            (project_id,)
        )
        sessions = self.cursor.fetchall()
        
        count = 0
        for session in sessions:
            source_path = pattern.format(
                session_name=session['session_name'],
                source_root=source_root
            )
            source_path = f"{source_root}/{source_path}" if not source_path.startswith(source_root) else source_path
            
            self.cursor.execute(
                "UPDATE data_sessions SET source_data_path = %s WHERE session_id = %s",
                (source_path, session['session_id'])
            )
            count += 1
        
        self.conn.commit()
        print(f"✓ 已批量更新 {count} 个会话的原始数据路径")
    
    def list_all_sources(self):
        """列出所有会话的数据来源信息"""
        self.cursor.execute("""
            SELECT 
                ds.session_id,
                ds.session_name,
                ds.session_path as processed_path,
                ds.source_data_path as original_path,
                p.project_name,
                p.source_data_root as project_root
            FROM data_sessions ds
            LEFT JOIN projects p ON ds.project_id = p.project_id
            ORDER BY ds.session_id
        """)
        return self.cursor.fetchall()
    
    def list_missing_sources(self):
        """列出缺失原始数据路径的会话"""
        self.cursor.execute("""
            SELECT 
                session_id,
                session_name,
                session_path
            FROM data_sessions
            WHERE source_data_path IS NULL
            ORDER BY session_id
        """)
        return self.cursor.fetchall()
    
    def get_session_info(self, session_id: int):
        """获取会话的数据来源信息"""
        self.cursor.execute("""
            SELECT 
                ds.session_id,
                ds.session_name,
                ds.session_path as processed_path,
                ds.source_data_path as original_path,
                p.project_name,
                p.source_data_root as project_root,
                ds.start_time,
                ds.end_time
            FROM data_sessions ds
            LEFT JOIN projects p ON ds.project_id = p.project_id
            WHERE ds.session_id = %s
        """, (session_id,))
        return self.cursor.fetchone()
    
    def set_default_source_paths(self, default_root: str = "jmsroot@192.168.2.78:/data/dwm_raw_data", pattern: str = "{session_name}"):
        """
        为所有未设置source_data_path的会话设置默认值
        
        Args:
            default_root: 默认的原始数据根目录
            pattern: 路径模式，可使用 {session_name} 占位符
        """
        # 获取所有未设置source_data_path的会话
        self.cursor.execute("""
            SELECT session_id, session_name 
            FROM data_sessions 
            WHERE source_data_path IS NULL OR source_data_path = ''
        """)
        sessions = self.cursor.fetchall()
        
        if not sessions:
            print("所有会话都已设置原始数据路径")
            return 0
        
        count = 0
        for session in sessions:
            # 构造路径
            source_path = pattern.format(session_name=session['session_name'])
            full_path = f"{default_root}/{source_path}" if not source_path.startswith('/') else source_path
            print(f"设置会话 ID {session['session_id']} 的原始数据路径为: {full_path}")
            
            self.cursor.execute(
                "UPDATE data_sessions SET source_data_path = %s WHERE session_id = %s",
                (full_path, session['session_id'])
            )
            count += 1
        
        self.conn.commit()
        print(f"✓ 已为 {count} 个会话设置默认原始数据路径")
        print(f"  默认根目录: {default_root}")
        print(f"  路径模式: {pattern}")
        return count
    
    def update_all_source_paths(self, default_root: str = "jmsroot@192.168.2.78:/data/dwm_raw_data", pattern: str = "{session_name}"):
        """
        更新所有会话的source_data_path（包括已设置的）
        
        Args:
            default_root: 原始数据根目录
            pattern: 路径模式，可使用 {session_name} 占位符
        """
        # 获取所有会话
        self.cursor.execute("""
            SELECT session_id, session_name 
            FROM data_sessions
        """)
        sessions = self.cursor.fetchall()
        
        if not sessions:
            print("没有找到会话")
            return 0
        
        count = 0
        for session in sessions:
            # 构造路径
            source_path = pattern.format(session_name=session['session_name'])
            # full_path = f"{default_root}/{source_path}" if not source_path.startswith('/') else source_path
            full_path = default_root
            print(f"更新会话 ID {session['session_id']} ({session['session_name']}) 的原始数据路径为: {full_path}")
            
            self.cursor.execute(
                "UPDATE data_sessions SET source_data_path = %s WHERE session_id = %s",
                (full_path, session['session_id'])
            )
            count += 1
        
        self.conn.commit()
        print(f"✓ 已更新 {count} 个会话的原始数据路径")
        print(f"  根目录: {default_root}")
        print(f"  路径模式: {pattern}")
        return count


def print_source_table(sources):
    """打印数据来源表格"""
    if not sources:
        print("没有数据")
        return
    
    print("\n" + "="*120)
    print(f"{'ID':<6} {'会话名称':<30} {'项目':<20} {'原始数据路径':<50}")
    print("-"*120)
    
    for src in sources:
        original = src.get('original_path') or src.get('source_data_path') or '(未设置)'
        print(f"{src['session_id']:<6} {src['session_name']:<30} "
              f"{src.get('project_name', 'N/A'):<20} {original:<50}")
    
    print("="*120)


def main():
    parser = argparse.ArgumentParser(
        description='数据来源路径管理工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 列出所有会话的数据来源
  python 06_manage_source_paths.py --list
  
  # 列出缺失原始路径的会话
  python 06_manage_source_paths.py --list-missing
  
  # 为所有未设置的会话设置默认原始数据路径
  python 06_manage_source_paths.py --set-default
  
  # 更新所有会话的原始数据路径（包括已设置的）
  python 06_manage_source_paths.py --update-all
  
  # 自定义默认根目录和路径模式
  python 06_manage_source_paths.py --update-all --default-root "user@host:/path" --pattern "{session_name}"
  
  # 更新项目的原始数据根目录
  python 06_manage_source_paths.py --update-project 1 --path /path/to/original/data
  
  # 更新单个会话的原始数据路径
  python 06_manage_source_paths.py --update-session 1 --path /path/to/bag/file.db3
  
  # 通过会话名称更新
  python 06_manage_source_paths.py --update-by-name gtxc_20251122_0 --path /path/to/bag.db3
  
  # 批量更新（使用模式）
  python 06_manage_source_paths.py --batch-update 1 --root /original/data --pattern "{session_name}.db3"
  
  # 查看单个会话信息
  python 06_manage_source_paths.py --info 1
        """
    )
    
    # 查询操作
    parser.add_argument('--list', action='store_true', help='列出所有会话的数据来源')
    parser.add_argument('--list-missing', action='store_true', help='列出缺失原始路径的会话')
    parser.add_argument('--info', type=int, metavar='SESSION_ID', help='查看指定会话的信息')
    
    # 更新操作
    parser.add_argument('--update-project', type=int, metavar='PROJECT_ID', help='更新项目的原始数据根目录')
    parser.add_argument('--update-session', type=int, metavar='SESSION_ID', help='更新会话的原始数据路径')
    parser.add_argument('--update-by-name', type=str, metavar='SESSION_NAME', help='通过会话名称更新原始数据路径')
    parser.add_argument('--batch-update', type=int, metavar='PROJECT_ID', help='批量更新项目下所有会话的原始路径')
    parser.add_argument('--set-default', action='store_true', help='为所有未设置的会话设置默认原始数据路径')
    parser.add_argument('--update-all', action='store_true', help='更新所有会话的原始数据路径（包括已设置的）')
    
    # 路径参数
    parser.add_argument('--path', type=str, help='原始数据路径')
    parser.add_argument('--root', type=str, help='原始数据根目录（用于批量更新）')
    parser.add_argument('--default-root', type=str, default='jmsroot@192.168.2.78:/data/dwm_raw_data',
                       help='默认原始数据根目录（默认: jmsroot@192.168.2.78:/data/dwm_raw_data）')
    parser.add_argument('--pattern', type=str, default='{session_name}', 
                       help='路径模式，可使用 {session_name} 占位符（默认: {session_name}）')
    
    args = parser.parse_args()
    
    with SourcePathManager(db_config) as manager:
        # 列出所有
        if args.list:
            sources = manager.list_all_sources()
            print(f"\n找到 {len(sources)} 个会话")
            print_source_table(sources)
        
        # 列出缺失的
        elif args.list_missing:
            sources = manager.list_missing_sources()
            print(f"\n找到 {len(sources)} 个缺失原始路径的会话")
            if sources:
                print("\n" + "="*80)
                print(f"{'ID':<6} {'会话名称':<40} {'处理后路径':<50}")
                print("-"*80)
                for src in sources:
                    print(f"{src['session_id']:<6} {src['session_name']:<40} {src['session_path']:<50}")
                print("="*80)
        
        # 查看单个会话信息
        elif args.info:
            info = manager.get_session_info(args.info)
            if info:
                print("\n" + "="*80)
                print(f"会话信息 (ID: {info['session_id']})")
                print("-"*80)
                print(f"会话名称: {info['session_name']}")
                print(f"项目名称: {info['project_name']}")
                print(f"项目原始数据根目录: {info['project_root'] or '(未设置)'}")
                print(f"处理后数据路径: {info['processed_path']}")
                print(f"原始数据路径: {info['original_path'] or '(未设置)'}")
                if info['start_time']:
                    print(f"开始时间: {info['start_time']}")
                    print(f"结束时间: {info['end_time']}")
                print("="*80)
            else:
                print(f"✗ 未找到会话 ID: {args.info}")
        
        # 更新项目
        elif args.update_project:
            if not args.path:
                print("错误: 需要指定 --path 参数")
                sys.exit(1)
            manager.update_project_source_root(args.update_project, args.path)
        
        # 更新会话
        elif args.update_session:
            if not args.path:
                print("错误: 需要指定 --path 参数")
                sys.exit(1)
            manager.update_session_source_path(args.update_session, args.path)
        
        # 通过名称更新
        elif args.update_by_name:
            if not args.path:
                print("错误: 需要指定 --path 参数")
                sys.exit(1)
            manager.update_session_source_path_by_name(args.update_by_name, args.path)
        
        # 批量更新
        elif args.batch_update:
            if not args.root:
                print("错误: 需要指定 --root 参数")
                sys.exit(1)
            manager.batch_update_sessions_by_pattern(args.batch_update, args.root, args.pattern)
        
        # 设置默认值
        elif args.set_default:
            count = manager.set_default_source_paths(args.default_root, args.pattern)
            if count > 0:
                print(f"\n可以使用 --list 查看更新后的结果")
        
        # 更新所有路径
        elif args.update_all:
            print(f"\n警告: 这将更新所有会话的原始数据路径！")
            confirm = input("确认继续? (y/n): ").strip().lower()
            if confirm == 'y':
                count = manager.update_all_source_paths(args.default_root, args.pattern)
                if count > 0:
                    print(f"\n可以使用 --list 查看更新后的结果")
            else:
                print("已取消")
        
        else:
            parser.print_help()


if __name__ == "__main__":
    main()
