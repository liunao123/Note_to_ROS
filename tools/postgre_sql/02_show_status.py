#!/usr/bin/env python3
"""数据库状态查看脚本（交互式）"""

import sys
import psycopg2
from psycopg2.extras import RealDictCursor
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
    sample_rows = results if max_rows is None else results[:max_rows]
    for row in sample_rows:
        for i, val in enumerate(row.values() if isinstance(row, dict) else row):
            col_widths[i] = max(col_widths[i], len(str(val)))
    
    # 限制列宽
    col_widths = [min(w, 50) for w in col_widths]
    
    # 打印表头
    header = " | ".join(name.ljust(width) for name, width in zip(col_names, col_widths))
    print(f"  {header}")
    print(f"  {'-' * len(header)}")
    
    # 打印数据行
    rows_to_print = results if max_rows is None else results[:max_rows]
    for row in rows_to_print:
        values = row.values() if isinstance(row, dict) else row
        line = " | ".join(str(v)[:width].ljust(width) for v, width in zip(values, col_widths))
        print(f"  {line}")
    
    if max_rows is not None and len(results) > max_rows:
        print(f"  ... (还有 {len(results) - max_rows} 行)")
    
    print(f"\n  总计: {len(results)} 行")


def _table_exists(cursor, table_name: str, schema: str = "public") -> bool:
    cursor.execute(
        """
        SELECT 1
        FROM information_schema.tables
        WHERE table_schema = %s AND table_name = %s
        LIMIT 1;
        """,
        (schema, table_name),
    )
    return cursor.fetchone() is not None


def _prompt(text: str, default: str | None = None) -> str:
    if default is None:
        suffix = ": "
    else:
        suffix = f" (默认 {default}): "
    val = input(text + suffix).strip()
    return val if val else (default or "")


def _prompt_int(text: str, default: int, min_value: int | None = None) -> int:
    while True:
        raw = _prompt(text, str(default))
        try:
            value = int(raw)
        except ValueError:
            print("  ❌ 请输入整数")
            continue
        if min_value is not None and value < min_value:
            print(f"  ❌ 请输入 >= {min_value} 的整数")
            continue
        return value


def _prompt_optional_int(text: str, default: int | None = None, min_value: int | None = None) -> int | None:
    """允许留空返回 None（通常表示“不限制/全部显示”）。"""
    hint = "" if default is None else str(default)
    raw = _prompt(text, hint).strip()
    if raw == "":
        return None
    while True:
        try:
            value = int(raw)
        except ValueError:
            print("  ❌ 请输入整数，或直接回车表示全部")
            raw = _prompt(text, hint).strip()
            if raw == "":
                return None
            continue
        if min_value is not None and value < min_value:
            print(f"  ❌ 请输入 >= {min_value} 的整数")
            raw = _prompt(text, hint).strip()
            if raw == "":
                return None
            continue
        return value


def _print_connection_info(cursor):
    print_header("🔌 连接信息")
    cursor.execute(
        """
        SELECT
            current_user AS current_user,
            current_database() AS current_database,
            version() AS version,
            pg_backend_pid() AS backend_pid,
            inet_server_addr() AS server_addr,
            inet_server_port() AS server_port;
        """
    )
    info = cursor.fetchone() or {}

    postgis_version = None
    try:
        cursor.execute("SELECT PostGIS_Version() AS postgis_version;")
        postgis_version = (cursor.fetchone() or {}).get("postgis_version")
    except Exception:
        postgis_version = None

    def show_kv(k, v):
        if v is None:
            v = "(未知)"
        print(f"  {k}: {v}")

    show_kv("User", info.get("current_user"))
    show_kv("Database", info.get("current_database"))
    show_kv("Backend PID", info.get("backend_pid"))
    ver = info.get("version")
    show_kv("PostgreSQL", ver.split(",")[0] if isinstance(ver, str) else ver)
    if postgis_version:
        show_kv("PostGIS", postgis_version)

    # unix socket 目录仅对本地连接更有意义；这里也一并展示
    try:
        cursor.execute("SHOW unix_socket_directories;")
        usd = (cursor.fetchone() or {}).get("unix_socket_directories")
        show_kv("unix_socket_directories", usd)
    except Exception:
        pass

    show_kv("server_addr", info.get("server_addr"))
    show_kv("server_port", info.get("server_port"))


def _query_overview(cursor):
    print_header("📊 数据库概览")
    metrics: list[tuple[str, str]] = [
        ("projects", "项目数"),
        ("data_sessions", "会话数"),
        ("sensors", "传感器数"),
        ("vehicle_poses", "位姿数"),
        ("images", "图像数"),
        ("point_clouds", "点云数"),
        ("submaps", "子地图数"),
        ("labels", "标注数"),
    ]

    parts = []
    for table, alias in metrics:
        if _table_exists(cursor, table):
            parts.append(f'(SELECT COUNT(*) FROM {table}) as "{alias}"')

    if not parts:
        print("  (public schema 下未找到可统计的表)")
        return

    query = "SELECT\n    " + ",\n    ".join(parts) + ";"
    print_table(cursor, query)


def _query_counts(cursor):
    print_header("🔢 计数统计")
    _query_overview(cursor)

    table_name = _prompt("  额外按表名查看 COUNT(*)（留空跳过）", default="").strip()
    if not table_name:
        return

    if not _table_exists(cursor, table_name):
        print(f"  ❌ 表不存在: {table_name}")
        return

    print_table(cursor, f'SELECT COUNT(*) AS "{table_name}_count" FROM {table_name};')


def _query_labels_menu(cursor):
    if not _table_exists(cursor, "labels"):
        print_header("🏷️ Labels")
        print("  ❌ labels 表不存在")
        print("  你可以先执行 sql/add_labels_table.sql 创建表，然后重新导入 labels 数据")
        return

    while True:
        print("\n" + "-" * 70)
        print("  a) labels 概览")
        print("  b) 按 label_name 统计 (TOP N)")
        print("  c) labels 明细（最近 N 条，可按 session_id 过滤）")
        print("  q) 返回主菜单")
        sub = _prompt("选择").lower()

        if sub in ("q", "quit", "exit"):
            return
        elif sub == "a":
            print_header("🏷️ Labels 概览")
            print_table(
                cursor,
                """
                SELECT
                    COUNT(*) AS "标注总数",
                    COUNT(DISTINCT session_id) AS "涉及会话数",
                    COUNT(DISTINCT label_name) AS "label_name种类",
                    MIN(timestamp_sec) AS "最早timestamp_sec",
                    MAX(timestamp_sec) AS "最晚timestamp_sec"
                FROM labels;
                """,
            )
        elif sub == "b":
            top_n = _prompt_int("TOP N", default=20, min_value=1)
            print_header("🏷️ Labels 统计（按 label_name）")
            print_table(
                cursor,
                f"""
                SELECT
                    label_name AS "label_name",
                    COUNT(*) AS "数量"
                FROM labels
                GROUP BY label_name
                ORDER BY COUNT(*) DESC
                LIMIT {int(top_n)};
                """,
                max_rows=top_n,
            )
        elif sub == "c":
            raw_sid = _prompt("session_id（留空=不过滤）", default="").strip()
            session_id = int(raw_sid) if raw_sid else None
            limit = _prompt_int("显示多少条", default=50, min_value=1)
            where = "" if session_id is None else f"WHERE session_id = {int(session_id)}"
            print_header("🏷️ Labels 明细")
            print_table(
                cursor,
                f"""
                SELECT
                    label_id AS "ID",
                    session_id AS "session_id",
                    frame_id AS "frame_id",
                    timestamp_sec AS "timestamp_sec",
                    label_name AS "label_name",
                    LEFT(relative_path, 60) AS "relative_path"
                FROM labels
                {where}
                ORDER BY timestamp_sec DESC, label_id DESC
                LIMIT {int(limit)};
                """,
                max_rows=limit,
            )
        else:
            print("  ❌ 无效选择")


def _query_projects(cursor, max_rows: int):
    print_header("📁 项目列表")
    print_table(
        cursor,
        """
            SELECT
                project_id as "ID",
                project_name as "项目名称",
                location as "位置",
                TO_CHAR(created_at, 'YYYY-MM-DD HH24:MI:SS') as "创建时间",
                description as "描述"
            FROM projects
            ORDER BY project_id;
        """,
        max_rows=max_rows,
    )


def _query_sessions(cursor, max_rows: int):
    print_header("🗂️  数据采集会话")
    print_table(
        cursor,
        """
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
        """,
        max_rows=max_rows,
    )


def _query_sensors(cursor, max_rows: int):
    print_header("📷 传感器列表")
    print_table(
        cursor,
        """
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
        """,
        max_rows=max_rows,
    )


def _query_distribution(cursor, max_rows: int):
    print_header("📈 数据分布（按会话和传感器）")
    print_table(
        cursor,
        """
            SELECT
                ds.session_name as "会话",
                s.sensor_name as "传感器",
                COUNT(*) as "数据量"
            FROM images i
            JOIN data_sessions ds ON i.session_id = ds.session_id
            JOIN sensors s ON i.sensor_id = s.sensor_id
            GROUP BY ds.session_id, ds.session_name, s.sensor_name
            ORDER BY ds.session_id, s.sensor_name;
        """,
        max_rows=max_rows,
    )


def _query_table_sizes(cursor, top_n: int):
    print_header("💾 数据库存储空间")
    print_table(
        cursor,
        f"""
            SELECT
                tablename as "表名",
                pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) AS "大小"
            FROM pg_tables
            WHERE schemaname = 'public'
            ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC
            LIMIT {int(top_n)};
        """,
        max_rows=top_n,
    )


def _query_file_path_samples(cursor):
    print_header("📂 文件路径示例")
    print_table(
        cursor,
        """
            SELECT
                'images' as "数据类型",
                s.sensor_name as "传感器",
                i.image_name as "文件名",
                LEFT(i.relative_path, 50) as "相对路径"
            FROM images i
            JOIN sensors s ON i.sensor_id = s.sensor_id
            ORDER BY i.image_id
            LIMIT 5;
        """,
        max_rows=10,
    )

    print_table(
        cursor,
        """
            SELECT
                'point_clouds' as "数据类型",
                s.sensor_name as "传感器",
                pc.cloud_name as "文件名",
                LEFT(pc.relative_path, 50) as "相对路径"
            FROM point_clouds pc
            JOIN sensors s ON pc.sensor_id = s.sensor_id
            ORDER BY pc.cloud_id
            LIMIT 5;
        """,
        max_rows=10,
    )

    print_table(
        cursor,
        """
            SELECT
                'submaps' as "数据类型",
                '-' as "传感器",
                sm.submap_name as "文件名",
                LEFT(sm.relative_path, 50) as "相对路径"
            FROM submaps sm
            ORDER BY sm.submap_id
            LIMIT 5;
        """,
        max_rows=10,
    )


def _query_recent_poses(cursor, session_id: int | None, limit: int):
    print_header("🗺️  最近位姿")
    if session_id is None:
        cursor.execute("SELECT MAX(session_id) AS max_session_id FROM vehicle_poses;")
        session_id = (cursor.fetchone() or {}).get("max_session_id")

    if not session_id:
        print("  (vehicle_poses 无数据)")
        return

    print(f"  使用 session_id = {session_id}")
    print_table(
        cursor,
        f"""
            SELECT
                frame_id as "帧ID",
                ROUND(position_x::numeric, 2) as "X",
                ROUND(position_y::numeric, 2) as "Y",
                ROUND(position_z::numeric, 2) as "Z",
                ROUND(latitude::numeric, 6) as "纬度",
                ROUND(longitude::numeric, 6) as "经度",
                ROUND(heading::numeric, 1) as "航向"
            FROM vehicle_poses
            WHERE session_id = {int(session_id)}
            ORDER BY frame_id DESC
            LIMIT {int(limit)};
        """,
        max_rows=limit,
    )


def main():
    try:
        conn = psycopg2.connect(**db_config, cursor_factory=RealDictCursor)
        # 本脚本只做只读查询：启用 autocommit 避免单条语句报错导致事务进入 aborted 状态
        conn.autocommit = True
        cursor = conn.cursor()
        
        print("\n" + "█"*70)
        print("█" + " "*68 + "█")
        print("█" + "移动测绘系统数据库状态".center(57) + "█")
        print("█" + " "*68 + "█")
        print("█"*70)
        
        _print_connection_info(cursor)

        # 非交互环境：默认只打印概览然后退出（方便脚本/CI）
        if not sys.stdin.isatty():
            _query_overview(cursor)
            cursor.close()
            conn.close()
            return

        # 交互模式
        print_header("✅ 连接成功")
        print("  请选择要查看的内容（输入序号，或 q 退出）")

        while True:
            print("\n" + "-" * 70)
            print("  1) 数据库概览（计数）")
            print("  2) 项目列表")
            print("  3) 会话列表")
            print("  4) 传感器列表")
            print("  5) 数据分布（会话×传感器）")
            print("  6) 表空间占用（TOP N）")
            print("  7) 文件路径示例")
            print("  8) 最近位姿（可指定 session_id）")
            print("  9) Labels（标注）")
            print(" 10) 计数统计（含点云数/标注数，可按表名）")
            print("  q) 退出")

            choice = _prompt("选择").lower()

            try:
                if choice in ("q", "quit", "exit"):
                    break
                elif choice == "1":
                    _query_overview(cursor)
                elif choice == "2":
                    max_rows = _prompt_optional_int("最多显示多少行（留空=全部）", default=None, min_value=1)
                    _query_projects(cursor, max_rows=max_rows)
                elif choice == "3":
                    max_rows = _prompt_optional_int("最多显示多少行（留空=全部）", default=None, min_value=1)
                    _query_sessions(cursor, max_rows=max_rows)
                elif choice == "4":
                    max_rows = _prompt_optional_int("最多显示多少行（留空=全部）", default=None, min_value=1)
                    _query_sensors(cursor, max_rows=max_rows)
                elif choice == "5":
                    max_rows = _prompt_optional_int("最多显示多少行（留空=全部）", default=None, min_value=1)
                    _query_distribution(cursor, max_rows=max_rows)
                elif choice == "6":
                    top_n = _prompt_int("TOP N", default=15, min_value=1)
                    _query_table_sizes(cursor, top_n=top_n)
                elif choice == "7":
                    _query_file_path_samples(cursor)
                elif choice == "8":
                    raw_sid = _prompt("session_id（留空=自动取最新）", default="").strip()
                    session_id = int(raw_sid) if raw_sid else None
                    limit = _prompt_int("显示多少条", default=10, min_value=1)
                    _query_recent_poses(cursor, session_id=session_id, limit=limit)
                elif choice == "9":
                    _query_labels_menu(cursor)
                elif choice == "10":
                    _query_counts(cursor)
                else:
                    print("  ❌ 无效选择")
            except psycopg2.Error as e:
                print(f"  ❌ 查询失败: {e}")
                # 若在非 autocommit 下发生错误，需 rollback 才能继续执行后续查询
                try:
                    if not getattr(conn, "autocommit", False):
                        conn.rollback()
                except Exception:
                    pass
        
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
        print("\n💡 提示:")
        print("  - 当前 config.yaml 未设置 host/port，会使用 Unix socket 连接。")
        print("  - 如果你的系统 socket 不在 /tmp，可在 config.yaml 设置 host: /var/run/postgresql")
        print("    或改为 TCP: host: localhost, port: 5432")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
