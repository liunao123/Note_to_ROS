#!/usr/bin/env python3
"""
位姿数据导出工具
将搜索到的位姿及其关联的所有文件复制到单独的文件夹
保持原有的目录结构
"""

import os
import sys
import shutil
import json
import argparse
import yaml
from pathlib import Path
from typing import List, Dict
import logging

# 导入搜索工具
# 动态导入，避免循环依赖
import importlib.util
search_poses_path = Path(__file__).parent / "qt" / "03_search_poses.py"
if not search_poses_path.exists():
    # 如果qt目录下不存在，尝试当前目录
    search_poses_path = Path(__file__).parent / "03_search_poses.py"
    
if not search_poses_path.exists():
    raise FileNotFoundError(f"找不到搜索模块: {search_poses_path}")

spec = importlib.util.spec_from_file_location("search_poses", str(search_poses_path))
search_poses_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(search_poses_module)
PoseSearcher = search_poses_module.PoseSearcher
db_config = search_poses_module.db_config

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


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
    
    logger.info(f"已加载配置文件: {config_path}")
    return config


# 加载配置
CONFIG = load_config()


class PoseExporter:
    """位姿数据导出器"""
    
    def __init__(self, output_root: Path):
        """
        初始化导出器
        
        Args:
            output_root: 输出根目录
        """
        self.output_root = Path(output_root)
        self.copied_files = set()  # 记录已复制的文件，避免重复
        self._session_paths = set()  # 会话路径集合，用于复制 calib
        self.stats = {
            'poses': 0,
            'odoms': 0,
            'optimized_poses': 0,
            'pointclouds': 0,
            'images': 0,
            'total_size': 0
        }
    
    def export_poses(self, poses: List[Dict], session_name: str = "exported_session"):
        """
        导出位姿数据
        
        Args:
            poses: 位姿列表（来自search_poses_by_distance的结果）
            session_name: 导出的会话名称
        """
        if not poses:
            logger.warning("没有位姿数据需要导出")
            return
        
        logger.info(f"开始导出 {len(poses)} 个位姿数据到: {self.output_root}")
        
        # 创建输出目录结构
        session_dir = self.output_root / session_name
        self._create_directory_structure(session_dir)
        
        # 导出每个位姿的数据
        for i, pose in enumerate(poses, 1):
            logger.info(f"处理位姿 {i}/{len(poses)}: Frame {pose['frame_id']}")
            self._export_single_pose(pose, session_dir)

        # 导出标定文件
        self._export_calib_files(session_dir)

        # 生成导出报告
        self._generate_report(session_dir)
        
        logger.info("="*80)
        logger.info("导出完成!")
        logger.info(f"位姿数量: {self.stats['poses']}")
        logger.info(f"Odom文件: {self.stats['odoms']}")
        logger.info(f"优化位姿文件: {self.stats['optimized_poses']}")
        logger.info(f"点云文件: {self.stats['pointclouds']}")
        logger.info(f"图像文件: {self.stats['images']}")
        logger.info(f"总大小: {self._format_size(self.stats['total_size'])}")
        logger.info(f"输出目录: {session_dir}")
        logger.info("="*80)
    
    def _create_directory_structure(self, session_dir: Path):
        """创建目录结构"""
        dirs = [
            session_dir / "odoms",
            session_dir / "sparse" / "vehicle_geo_pose",
            session_dir / "pointclouds",
            session_dir / "images",
            session_dir / "calib",
        ]
        
        for d in dirs:
            d.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"创建目录结构: {session_dir}")
    
    def _export_single_pose(self, pose: Dict, session_dir: Path):
        """导出单个位姿的所有相关文件"""
        self.stats['poses'] += 1
        # 记录会话路径（用于后续复制 calib）
        if pose.get('session_path'):
            self._session_paths.add(pose.get('session_path'))
        
        # 1. 复制odom文件
        if pose.get('odom_path'):
            self._copy_file(
                pose['odom_path'],
                session_dir / "odoms" / Path(pose['odom_path']).name
            )
            self.stats['odoms'] += 1
        
        # 2. 复制优化位姿文件
        if pose.get('session_path') and pose.get('opt_timestamp_full'):
            opt_pose_path = f"{pose['session_path']}/sparse/vehicle_geo_pose/{pose['frame_id']}_{pose['opt_timestamp_full']:.3f}.yaml"
            self._copy_file(
                opt_pose_path,
                session_dir / "sparse" / "vehicle_geo_pose" / Path(opt_pose_path).name
            )
            self.stats['optimized_poses'] += 1
        
        # 3. 复制点云文件
        if pose.get('pointcloud_path'):
            self._copy_file(
                pose['pointcloud_path'],
                session_dir / "pointclouds" / Path(pose['pointcloud_path']).name
            )
            self.stats['pointclouds'] += 1
        
        # 4. 复制图像文件
        if pose.get('image_paths'):
            image_paths_dict = pose['image_paths']
            if isinstance(image_paths_dict, str):
                image_paths_dict = json.loads(image_paths_dict)
            
            for sensor_name, img_path in image_paths_dict.items():
                # 为每个传感器创建子目录
                sensor_dir = session_dir / "images" / sensor_name
                sensor_dir.mkdir(parents=True, exist_ok=True)
                
                self._copy_file(
                    img_path,
                    sensor_dir / Path(img_path).name
                )
                self.stats['images'] += 1
    
    def _copy_file(self, src: str, dst: Path):
        """
        复制文件
        
        Args:
            src: 源文件路径
            dst: 目标文件路径
        """
        src_path = Path(src)
        
        # 避免重复复制
        if str(src_path) in self.copied_files:
            return
        
        if not src_path.exists():
            logger.warning(f"源文件不存在: {src_path}")
            return
        
        try:
            # 确保目标目录存在
            dst.parent.mkdir(parents=True, exist_ok=True)
            
            # 复制文件
            shutil.copy2(src_path, dst)
            
            # 记录文件大小
            file_size = src_path.stat().st_size
            self.stats['total_size'] += file_size
            
            # 标记已复制
            self.copied_files.add(str(src_path))
            
            logger.debug(f"已复制: {src_path.name} ({self._format_size(file_size)})")
            
        except Exception as e:
            logger.error(f"复制文件失败 {src_path} -> {dst}: {e}")
    
    def _generate_report(self, session_dir: Path):
        """生成导出报告"""
        report = {
            'export_info': {
                'output_directory': str(session_dir),
                'total_poses': self.stats['poses'],
                'total_size_bytes': self.stats['total_size'],
                'total_size_human': self._format_size(self.stats['total_size'])
            },
            'file_counts': {
                'odoms': self.stats['odoms'],
                'optimized_poses': self.stats['optimized_poses'],
                'pointclouds': self.stats['pointclouds'],
                'images': self.stats['images']
            }
        }
        
        report_file = session_dir / "export_report.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        logger.info(f"生成导出报告: {report_file}")

    def _export_calib_files(self, session_dir: Path):
        """复制会话下的 calib 目录（只需复制一次）

        从第一个发现的 session_path 下复制 calib/* 到输出的 session_dir/calib
        已复制的文件由 self.copied_files 控制，避免重复复制。
        """
        if not self._session_paths:
            logger.info("没有发现任何会话路径，跳过复制 calib 文件")
            return

        # 只从第一个路径复制一次
        src_session = Path(next(iter(self._session_paths)))
        calib_src = src_session / 'calib'
        if not calib_src.exists():
            logger.info(f"未找到 calib 目录: {calib_src}，跳过")
            return

        calib_dst = session_dir / 'calib'
        files_copied = 0

        for root, dirs, files in os.walk(calib_src):
            rel = Path(root).relative_to(calib_src)
            target_dir = calib_dst / rel
            target_dir.mkdir(parents=True, exist_ok=True)

            for fname in files:
                srcf = Path(root) / fname
                dstf = target_dir / fname
                if str(srcf) in self.copied_files:
                    continue

                try:
                    shutil.copy2(srcf, dstf)
                    self.copied_files.add(str(srcf))
                    file_size = srcf.stat().st_size
                    self.stats['total_size'] += file_size
                    files_copied += 1
                except Exception as e:
                    logger.error(f"复制 calib 文件失败 {srcf} -> {dstf}: {e}")

        if files_copied:
            logger.info(f"已复制 {files_copied} 个 calib 文件到 {calib_dst}")
        else:
            logger.info("没有新的 calib 文件需要复制（或已被复制过）")
    
    @staticmethod
    def _format_size(size_bytes: int) -> str:
        """格式化文件大小"""
        for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
            if size_bytes < 1024.0:
                return f"{size_bytes:.2f} {unit}"
            size_bytes /= 1024.0
        return f"{size_bytes:.2f} PB"


def main():
    """主函数"""
    # 从配置文件获取默认值
    search_params = CONFIG.get('search_params', {})
    default_location = search_params.get('default_location', {})
    data_export = CONFIG.get('data_export', {})
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='位姿数据导出工具 - 将搜索到的位姿及其关联文件复制到单独的文件夹',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 使用默认配置
  python 03_search_poses_and_copy_files.py
  
  # 指定输出目录
  python 03_search_poses_and_copy_files.py -o /path/to/output
  
  # 指定输出目录和会话名称
  python 03_search_poses_and_copy_files.py -o /path/to/output -n my_session
  
  # 修改搜索参数
  python 03_search_poses_and_copy_files.py -o /tmp/export --lat 31.41 --lon 120.66 --distance 50
        """
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=data_export.get('root_dir', '/mnt/nvme0n1p2/project/postgresql/export/'),
        help=f'输出根目录 (默认: {data_export.get("root_dir", "/mnt/nvme0n1p2/project/postgresql/export/")})'
    )
    
    parser.add_argument(
        '-n', '--name',
        type=str,
        default=None,
        help='会话名称 (默认: 自动生成为 search_LAT_LON_DISTm)'
    )
    
    parser.add_argument(
        '--lat',
        type=float,
        default=default_location.get('latitude', 31.41033324),
        help=f'目标纬度 (默认: {default_location.get("latitude", 31.41033324)})'
    )
    
    parser.add_argument(
        '--lon',
        type=float,
        default=default_location.get('longitude', 120.65582103),
        help=f'目标经度 (默认: {default_location.get("longitude", 120.65582103)})'
    )
    
    parser.add_argument(
        '--distance',
        type=float,
        default=search_params.get('distance', 100.0),
        help=f'搜索半径(米) (默认: {search_params.get("distance", 100.0)})'
    )
    
    parser.add_argument(
        '--limit',
        type=int,
        default=search_params.get('limit'),
        help='限制导出的位姿数量 (默认: 无限制)'
    )
    
    args = parser.parse_args()
    
    print("\n" + "="*80)
    print("  位姿数据导出工具")
    print("="*80)
    
    # 搜索参数
    target_lon = args.lon
    target_lat = args.lat
    distance = args.distance
    
    # 输出目录
    output_root = Path(args.output)
    session_name = args.name if args.name else f"search_{target_lat:.6f}_{target_lon:.6f}_{distance}m"
    
    print(f"\n搜索参数:")
    print(f"  目标位置: ({target_lat:.6f}, {target_lon:.6f})")
    print(f"  搜索半径: {distance} 米")
    print(f"  UTM区号: 自动计算")
    if args.limit:
        print(f"  导出限制: {args.limit} 个位姿")
    print(f"\n输出配置:")
    print(f"  输出根目录: {output_root}")
    print(f"  会话名称: {session_name}")
    print(f"  完整路径: {output_root / session_name}")
    print("-" * 80)

    # 搜索位姿
    print("\n步骤1: 搜索位姿数据...")
    with PoseSearcher(db_config) as searcher:
        poses = searcher.search_poses_by_distance(
            longitude=target_lon,
            latitude=target_lat,
            distance_meters=distance,
            limit=args.limit
        )
    
    if not poses:
        print("未找到符合条件的位姿")
        return
    
    print(f"找到 {len(poses)} 个位姿")
    
    # 导出数据
    print("\n步骤2: 导出文件...")
    exporter = PoseExporter(output_root)
    exporter.export_poses(poses, session_name)
    
    print("\n完成！")


if __name__ == "__main__":
    main()
