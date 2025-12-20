#!/usr/bin/env python3
"""
数据库连接测试脚本
用于验证数据库配置是否正确
"""

import psycopg2
import sys
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
    
    print(f"已加载配置文件: {config_path}")
    return config

def test_connection(config_name, db_config):
    """测试数据库连接"""
    print(f"\n{'='*60}")
    print(f"测试配置: {config_name}")
    print(f"{'='*60}")
    print(f"  Host: {db_config.get('host', '(使用Unix socket)')}")
    print(f"  Port: {db_config.get('port', '(默认)')}")
    print(f"  Database: {db_config.get('database', '(未指定)')}")
    print(f"  User: {db_config.get('user', '(当前用户)')}")
    print(f"  Password: {'***' if db_config.get('password') else '(无)'}")
    
    try:
        # 尝试连接
        conn = psycopg2.connect(**db_config)
        cursor = conn.cursor()
        
        # 获取基本信息
        cursor.execute("""
            SELECT 
                current_user,
                current_database(),
                version(),
                pg_backend_pid()
        """)
        user, database, version, pid = cursor.fetchone()
        
        print(f"\n✅ 连接成功！")
        print(f"  当前用户: {user}")
        print(f"  当前数据库: {database}")
        print(f"  进程ID: {pid}")
        print(f"  PostgreSQL版本: {version.split(',')[0]}")
        
        # 测试PostGIS
        cursor.execute("SELECT PostGIS_Version();")
        postgis_version = cursor.fetchone()[0]
        print(f"  PostGIS版本: {postgis_version}")
        
        # 检查表
        cursor.execute("""
            SELECT COUNT(*) 
            FROM information_schema.tables 
            WHERE table_schema = 'public' 
            AND table_type = 'BASE TABLE'
            AND table_name NOT IN ('spatial_ref_sys', 'geography_columns', 
                                   'geometry_columns', 'raster_columns', 
                                   'raster_overviews');
        """)
        table_count = cursor.fetchone()[0]
        print(f"  数据表数量: {table_count}")
        
        cursor.close()
        conn.close()
        
        return True
        
    except psycopg2.OperationalError as e:
        print(f"\n❌ 连接失败！")
        print(f"  错误类型: OperationalError")
        print(f"  错误信息: {e}")
        
        # 提供解决建议
        error_str = str(e)
        print(f"\n💡 建议:")
        
        if "no password supplied" in error_str:
            print("  - PostgreSQL要求密码认证，但未提供密码")
            print("  - 解决方案1: 在配置中添加密码")
            print("  - 解决方案2: 不指定host，使用Unix socket连接")
            
        elif "authentication failed" in error_str:
            print("  - 认证失败，请检查用户名和密码")
            print("  - 确认PostgreSQL用户是否存在: psql -c '\\du'")
            
        elif "Connection refused" in error_str:
            print("  - PostgreSQL服务未运行或端口错误")
            print("  - 检查服务: systemctl status postgresql")
            print("  - 检查端口: netstat -tlnp | grep 5432")
            
        elif "database" in error_str and "does not exist" in error_str:
            print("  - 数据库不存在")
            print("  - 创建数据库: createdb gtxc_db")
            
        return False
        
    except Exception as e:
        print(f"\n❌ 连接失败！")
        print(f"  错误类型: {type(e).__name__}")
        print(f"  错误信息: {e}")
        return False


def main():
    """主函数"""
    print("\n" + "█"*60)
    print("█" + " "*58 + "█")
    print("█" + "  PostgreSQL 数据库连接测试".center(58) + "█")
    print("█" + " "*58 + "█")
    print("█"*60 + "\n")
    
    # 加载配置文件
    try:
        config = load_config()
        db_settings = config.get('database', {})
    except FileNotFoundError as e:
        print(f"❌ 错误: {e}")
        print("\n请确保 config.yaml 文件存在于脚本所在目录")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 加载配置文件失败: {e}")
        sys.exit(1)
    
    # 从配置文件构建数据库配置
    db_config = {
        'database': db_settings.get('name', 'gtxc_db'),
        'user': db_settings.get('user', 'tyjt')
    }
    
    # 如果配置了 host 和 port，添加到配置中
    if db_settings.get('host'):
        db_config['host'] = db_settings['host']
    if db_settings.get('port'):
        db_config['port'] = db_settings['port']
    if db_settings.get('password'):
        db_config['password'] = db_settings['password']
    
    print(f"\n从配置文件读取的数据库设置:")
    print(f"  Database: {db_config.get('database')}")
    print(f"  User: {db_config.get('user')}")
    if 'host' in db_config:
        print(f"  Host: {db_config['host']}")
        print(f"  Port: {db_config.get('port', 5432)}")
    else:
        print(f"  连接方式: Unix Socket (本地连接)")
    
    # 测试连接
    success = test_connection("从 config.yaml 读取的配置", db_config)
    
    # 总结
    print(f"\n{'='*60}")
    if success:
        print("✅ 数据库连接测试成功！")
        print(f"{'='*60}")
        print("\n配置正确，可以使用以下脚本：")
        print("  - 01_data_importer.py      # 数据导入")
        print("  - 02_show_status.py        # 查看状态")
        print("  - 03_search_poses_and_copy_files.py  # 搜索和导出")
        print("  - 03_merge_session_data.py # 合并点云")
    else:
        print("❌ 数据库连接测试失败！")
        print(f"{'='*60}")
        print("\n请检查:")
        print("  1. PostgreSQL服务是否运行: systemctl status postgresql")
        print("  2. 数据库是否存在: psql -l | grep gtxc_db")
        print("  3. 用户权限是否正确: psql -c '\\du'")
        print("  4. config.yaml 中的配置是否正确")
        print("\n配置文件位置: config.yaml")
        sys.exit(1)
    
    print("\n" + "█"*60 + "\n")


if __name__ == "__main__":
    main()
