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
from pathlib import Path
from typing import List, Dict
import logging
from qt.load_config import load_config

# 导入3D box投影工具函数
from qt.box_projection import (
    clip_hd_map_by_config
)

# 导入搜索工具
# 动态导入，避免循环依赖
import importlib.util
search_poses_path = Path(__file__).parent / "qt" / "search_poses.py"
if not search_poses_path.exists():
    # 如果qt目录下不存在，尝试当前目录
    search_poses_path = Path(__file__).parent / "search_poses.py"
    
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

class PoseExporter:
    """位姿数据导出器"""
    
    def __init__(self, output_root: Path, *, copy_options: Dict[str, bool] | None = None):
        """
        初始化导出器
        
        Args:
            output_root: 输出根目录
            copy_options: 复制开关配置，例如:
                {
                    'copy_images': True,
                    'copy_pointclouds': True,
                    'copy_odoms': True,
                    'copy_optimized_poses': True,
                    'copy_labels': True,
                }
        """
        self.output_root = Path(output_root)
        self.copy_options = {
            'copy_images': True,
            'copy_pointclouds': True,
            'copy_odoms': True,
            'copy_optimized_poses': True,
            'copy_labels': True,
            'copy_masks': True,
        }
        if copy_options:
            self.copy_options.update({k: bool(v) for k, v in copy_options.items()})
        self.copied_files = set()  # 记录已复制的文件，避免重复
        self._session_paths = set()  # 会话路径集合，用于复制 calib
        self._copied_static_masks = False  # 记录已复制的static masks目录
        self.stats = {
            'poses': 0,
            'odoms': 0,
            'optimized_poses': 0,
            'pointclouds': 0,
            'images': 0,
            'labels': 0,
            'masks': 0,
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
        logger.info(
            "复制开关: images=%s, pointclouds=%s, odoms=%s, optimized_poses=%s, labels=%s, masks=%s",
            self.copy_options.get('copy_images', True),
            self.copy_options.get('copy_pointclouds', True),
            self.copy_options.get('copy_odoms', True),
            self.copy_options.get('copy_optimized_poses', True),
            self.copy_options.get('copy_labels', True),
            self.copy_options.get('copy_masks', True),
        )
        
        # 创建输出目录结构
        session_dir = self.output_root / session_name
        self._create_directory_structure(session_dir)
        
        # 导出每个位姿的数据
        for i, pose in enumerate(poses, 1):
            if i % 100 == 0 or i == len(poses):
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
        logger.info(f"Labels文件: {self.stats['labels']}")
        logger.info(f"Masks文件: {self.stats['masks']}")
        logger.info(
            "复制开关(回显): images=%s, pointclouds=%s, odoms=%s, optimized_poses=%s, labels=%s, masks=%s",
            self.copy_options.get('copy_images', True),
            self.copy_options.get('copy_pointclouds', True),
            self.copy_options.get('copy_odoms', True),
            self.copy_options.get('copy_optimized_poses', True),
            self.copy_options.get('copy_labels', True),
            self.copy_options.get('copy_masks', True),
        )
        logger.info(f"总大小: {self._format_size(self.stats['total_size'])}")
        logger.info(f"输出目录: {session_dir}")
        logger.info("="*80)
        if not self._copied_static_masks:
            print("=" * 80)
            logger.warning("未找到 static masks 目录，未复制 static masks")
            print("=" * 80)


    def _create_directory_structure(self, session_dir: Path):
        """创建目录结构"""
        dirs = []
        if self.copy_options.get('copy_odoms', True):
            dirs.append(session_dir / "odoms")
        if self.copy_options.get('copy_optimized_poses', True):
            dirs.append(session_dir / "sparse" / "vehicle_geo_pose")
        if self.copy_options.get('copy_pointclouds', True):
            dirs.append(session_dir / "pointclouds")
        if self.copy_options.get('copy_images', True):
            dirs.append(session_dir / "images")
        if self.copy_options.get('copy_labels', True):
            dirs.append(session_dir / "labels")
        if self.copy_options.get('copy_masks', True):
            dirs.append(session_dir / "masks")
        # session_dir / "calib" 目录将在复制时创建
        
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
        if self.copy_options.get('copy_odoms', True) and pose.get('odom_path'):
            self._copy_file(
                pose['odom_path'],
                session_dir / "odoms" / Path(pose['odom_path']).name
            )
            self.stats['odoms'] += 1
        
        # 2. 复制优化位姿文件
        if self.copy_options.get('copy_optimized_poses', True) and pose.get('session_path') and pose.get('opt_timestamp_full'):
            opt_pose_path = f"{pose['session_path']}/sparse/vehicle_geo_pose/{pose['frame_id']}_{pose['opt_timestamp_full']:.3f}.yaml"
            self._copy_file(
                opt_pose_path,
                session_dir / "sparse" / "vehicle_geo_pose" / Path(opt_pose_path).name
            )
            self.stats['optimized_poses'] += 1
        
        # 3. 复制点云文件
        if self.copy_options.get('copy_pointclouds', True) and pose.get('pointcloud_path'):
            self._copy_file(
                pose['pointcloud_path'],
                session_dir / "pointclouds" / Path(pose['pointcloud_path']).name
            )
            self.stats['pointclouds'] += 1
        
        # 4. 复制图像文件
        if self.copy_options.get('copy_images', True) and pose.get('image_paths'):
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
        
        # 5. 复制labels文件
        if self.copy_options.get('copy_labels', True) and pose.get('session_path') and pose.get('frame_id') is not None:
            # 构建labels文件路径：session_path/labels/frame_id_timestamp.json
            # 从odom_path中提取时间戳部分（如果有odom_path）
            if pose.get('odom_path'):
                odom_filename = Path(pose['odom_path']).stem  # 如: 0_1763951779.000
                label_path = f"{pose['session_path']}/labels/{odom_filename}.json"
                gd_label_path = f"{pose['session_path']}/debug_files/3dbox/{odom_filename}_gd.json"
                
                self._copy_file(
                    label_path,
                    session_dir / "labels" / Path(label_path).name
                )
                self.stats['labels'] += 1
                
                self._copy_file(
                    gd_label_path,
                    session_dir / "debug_files" / "3dbox" / Path(gd_label_path).name
                )
                self.stats['labels'] += 1
        
        # 6. 复制masks文件（保持原始目录结构）
        if self.copy_options.get('copy_masks', True) and pose.get('session_path') and pose.get('image_paths'):
            session_path = pose['session_path']
            if not self._copied_static_masks:
                static_masks_dir = Path("/data/dwm_data/mask_static/static")
                if static_masks_dir.exists():
                    output_static_dir = session_dir / "masks/static" 
                    try:
                        shutil.copytree(static_masks_dir, output_static_dir, dirs_exist_ok=True)
                        self._copied_static_masks = True
                        print("已复制 static masks 目录 ")
                    except Exception as e:
                        logger.error(f"复制 static/masks 目录失败 {static_masks_dir} -> {output_static_dir}: {e}")
                        exit(1)
                else:
                    print("=" * 80)
                    print(f"[WARN] static masks 目录不存在: {static_masks_dir}, 跳过复制 static masks")
                    print("=" * 80) 
            
            # masks文件夹结构：session_path/masks/dynamic/{sensor_name}/{frame_id}_{timestamp}.png
            # 使用odom_path提取frame_id_timestamp部分
            odom_filename = Path(pose['odom_path']).stem  # 如: 0_1763951779.000
            mask_filename = f"{odom_filename}.png"  # 如: 0_1763951779.000.png
            
            image_paths_dict = pose['image_paths']
            if isinstance(image_paths_dict, str):
                image_paths_dict = json.loads(image_paths_dict)
            
            for sensor_name, img_path in image_paths_dict.items():
                # masks目录路径 - dynamic
                masks_sensor_dir = Path(session_path) / "masks" / "dynamic" / sensor_name
                mask_file_path = masks_sensor_dir / mask_filename
                # print(f"检查 mask 文件: {mask_file_path}")
                if mask_file_path.exists():
                    # 创建对应的输出子目录
                    target_dir = session_dir / "masks" / "dynamic" / sensor_name
                    target_dir.mkdir(parents=True, exist_ok=True)
                    # print(f"检查 mask 文件: {target_dir / mask_filename}")
                    self._copy_file(
                        str(mask_file_path),
                        target_dir / mask_filename
                    )
                    self.stats['masks'] += 1
                
                # masks目录路径 - sky
                masks_sensor_dir_sky = Path(session_path) / "masks" / "sky" / sensor_name
                mask_file_path_sky = masks_sensor_dir_sky / mask_filename
                # print(f"检查 sky mask 文件: {mask_file_path_sky}")
                if mask_file_path_sky.exists():
                    # 创建对应的输出子目录
                    target_dir_sky = session_dir / "masks" / "sky" / sensor_name
                    target_dir_sky.mkdir(parents=True, exist_ok=True)
                    self._copy_file(
                        str(mask_file_path_sky),
                        target_dir_sky / mask_filename
                    )
                    self.stats['masks'] += 1
                else:
                    print(f"[WARN] 未找到 sky mask 文件: {mask_file_path_sky}，跳过复制 sky mask")
    
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
                'images': self.stats['images'],
                'labels': self.stats['labels'],
                'masks': self.stats['masks']
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
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='位姿数据导出工具')
    parser.add_argument('--longitude', '--lon', type=float, default=None,
                        help='Target longitude (default: from config)')
    parser.add_argument('--latitude', '--lat', type=float, default=None,
                        help='Target latitude (default: from config)')
    parser.add_argument('--roi_distance_min', type=float, default=None,
                        help='ROI distance min in meters (default: from config)')
    parser.add_argument('--roi_distance_max', type=float, default=None,
                        help='ROI distance max in meters (default: from config)')
    args = parser.parse_args()
    
    # 从配置文件获取默认值
    # 加载配置
    config = load_config()
    search_params = config.get('search_params', {})
    default_location = search_params.get('default_location', {})
    
    # 获取会话名称列表（用于过滤搜索范围）
    search_session_names = search_params.get('use_session_subdir', [])
    print(f"搜索会话子目录: {search_session_names if search_session_names else '全部会话'}")

    # 直接使用配置文件中的参数
    print("\n" + "="*80)
    print("  位姿数据导出工具")
    print("="*80)

    # 搜索参数 - 命令行参数优先，否则使用配置文件的值
    target_lat = args.latitude if args.latitude is not None else default_location.get('latitude', 31.41033324)
    target_lon = args.longitude if args.longitude is not None else default_location.get('longitude', 120.65582103)
    roi_distance_max = args.roi_distance_max if args.roi_distance_max is not None else search_params.get('roi_distance_max', 100.0)
    roi_distance_min = args.roi_distance_min if args.roi_distance_min is not None else search_params.get('roi_distance_min', 50.0)
    limit = search_params.get('limit')

    # 复制开关（6个参数）
    copy_options = {
        'copy_images': search_params.get('copy_images', True),
        'copy_pointclouds': search_params.get('copy_pointclouds', True),
        'copy_odoms': search_params.get('copy_odoms', True),
        'copy_optimized_poses': search_params.get('copy_optimized_poses', True),
        'copy_labels': search_params.get('copy_labels', True),
        'copy_masks': search_params.get('copy_masks', True),
    }

    print("\n复制开关:")
    print(f"  copy_images: {copy_options['copy_images']}")
    print(f"  copy_pointclouds: {copy_options['copy_pointclouds']}")
    print(f"  copy_odoms: {copy_options['copy_odoms']}")
    print(f"  copy_optimized_poses: {copy_options['copy_optimized_poses']}")
    print(f"  copy_labels: {copy_options['copy_labels']}")
    print(f"  copy_masks: {copy_options['copy_masks']}")

    # 输出目录和会话名模板直接从 search_params 读取
    output_root = Path(search_params.get('export_dir', '/mnt/nvme0n2/project/postgresql/export/'))
    session_name_template = search_params.get('session_name_template', 'search_{lat:.6f}_{lon:.6f}_{roi_distance_min}_{roi_distance_max}m')
    session_name = session_name_template.format(lat=target_lat, lon=target_lon, roi_distance_min=roi_distance_min, roi_distance_max=roi_distance_max)

    print(f"\n搜索参数:")
    print(f"  目标位置: ({target_lat:.6f}, {target_lon:.6f})")
    print(f"  搜索半径: {roi_distance_max} 米")
    print(f"  UTM区号: 自动计算")
    if limit:
        print(f"  导出限制: {limit} 个位姿")
    print(f"\n输出配置:")
    print(f"  输出根目录: {output_root}")
    print(f"  会话名称: {session_name}")
    print(f"  完整路径: {output_root / session_name}")
    print("-" * 80)
    # exit()

    # 获取并裁切高精地图数据
    print("\n步骤0: 裁切高精地图数据...")
    hd_map_config = config.get('hd_map', {})
    input_folder = hd_map_config.get('input_folder')
    output_folder = str (output_root / session_name / "hd_map")

    print("input_folder : ", input_folder)
    print("output_folder : ", output_folder)
    if input_folder and os.path.exists(input_folder):
        clip_hd_map_by_config(target_lon, target_lat, roi_distance_max, input_folder, output_folder)
        print("高精地图裁切完成。\n")
    else:
        print("-" * 80)
        print(f"[WARN] 高精地图文件 {input_folder} 不存在，跳过高精地图裁切。")
        print("-" * 80)

    # 搜索位姿
    print("\n步骤1: 搜索位姿数据...")
    if search_session_names:
        print(f"  限制搜索范围到以下会话: {', '.join(search_session_names)}")
        
    with PoseSearcher(db_config) as searcher:
        poses = searcher.search_poses_by_distance(
            longitude=target_lon,
            latitude=target_lat,
            distance_meters=roi_distance_max,
            session_names=search_session_names if search_session_names else None,
            limit=limit
        )

    if not poses:
        print("未找到符合条件的位姿")
        return

    print(f"找到 {len(poses)} 个位姿")


    # 导出数据前，循环查找第一个存在calib目录的会话并复制
    copied = False
    if poses:
        for pose in poses:
            session_path_for_calib = pose.get('session_path')
            if not session_path_for_calib:
                continue
            src_calib = Path(session_path_for_calib) / 'calib'
            dst_calib = output_root / session_name / 'calib'
            if src_calib.exists():
                try:
                    if dst_calib.exists():
                        shutil.rmtree(dst_calib)
                    shutil.copytree(src_calib, dst_calib)
                    print(f"[INFO] 已复制calib目录: {src_calib} -> {dst_calib}")
                    copied = True
                    break
                except Exception as e:
                    print(f"[ERROR] 复制calib目录失败: {e}")
                    break
            else:
                print(f"[WARN] 源calib目录不存在: {src_calib}")
    if not copied:
        print("[WARN] 所有会话均未找到可用的calib目录，未复制calib")
    # 导出数据
    print("\n步骤2: 导出文件...")
    exporter = PoseExporter(output_root, copy_options=copy_options)
    exporter.export_poses(poses, session_name)

    print("\n完成！")


if __name__ == "__main__":
    main()