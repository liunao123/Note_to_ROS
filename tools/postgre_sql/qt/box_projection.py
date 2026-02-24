"""
3D box投影工具模块

包含3D box投影到相机图像平面的所有核心函数。
"""

import numpy as np
import cv2
import yaml
import os
from typing import Tuple, List, Optional, Union, Dict
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import geopandas as gpd
from shapely.geometry import Point
from pyproj import CRS, Transformer
from pathlib import Path


def get_all_session_dirs(base_dir):
    """
    获取基础目录下的所有子文件夹作为session目录
    
    Args:
        base_dir: 基础目录路径
        
    Returns:
        list: session目录列表
    """
    if not os.path.exists(base_dir):
        print(f'[ERROR] Base directory not found: {base_dir}')
        return []
    
    session_dirs = []
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        if os.path.isdir(item_path):
            session_dirs.append(item_path)
    
    return sorted(session_dirs)

# ==================== 坐标转换函数 ====================
def lonlat_to_utm(longitude: float, latitude: float, zone: Optional[int] = None) -> Tuple[float, float, int, str]:
    """
    将经纬度转换为UTM坐标
    
    Args:
        longitude: 经度 (度)
        latitude: 纬度 (度)
        zone: UTM分区号，如果为None则自动计算
    
    Returns:
        Tuple[float, float, int, str]: (UTM东向坐标, UTM北向坐标, UTM分区号, 半球('N'或'S'))
    
    Example:
        >>> x, y, zone, hemisphere = lonlat_to_utm(121.0, 31.0)
        >>> print(f"UTM Zone: {zone}{hemisphere}, X: {x:.2f}, Y: {y:.2f}")
    """
    # 如果没有指定zone，根据经度自动计算
    if zone is None:
        zone = int((longitude + 180) / 6) + 1
    
    # 判断南北半球
    hemisphere = 'N' if latitude >= 0 else 'S'
    
    # 创建WGS84到UTM的转换器
    # EPSG:4326 是 WGS84经纬度坐标系
    wgs84 = 'EPSG:4326'
    
    # 构建UTM的EPSG代码
    # 北半球: 32600 + zone, 南半球: 32700 + zone
    utm_epsg = f'EPSG:{32600 + zone if hemisphere == "N" else 32700 + zone}'
    
    # 创建转换器
    transformer = Transformer.from_crs(wgs84, utm_epsg, always_xy=True)
    
    # 转换坐标 (经度, 纬度) -> (东向, 北向)
    utm_x, utm_y = transformer.transform(longitude, latitude)
    
    return utm_x, utm_y, zone, hemisphere


def utm_to_lonlat(utm_x: float, utm_y: float, zone: int, hemisphere: str = 'N') -> Tuple[float, float]:
    """
    将UTM坐标转换为经纬度
    
    Args:
        utm_x: UTM东向坐标 (米)
        utm_y: UTM北向坐标 (米)
        zone: UTM分区号 (1-60)
        hemisphere: 半球 ('N' 或 'S')
    
    Returns:
        Tuple[float, float]: (经度, 纬度)
    
    Example:
        >>> lon, lat = utm_to_lonlat(700000, 3431234, 51, 'N')
        >>> print(f"Longitude: {lon:.6f}, Latitude: {lat:.6f}")
    """
    # 构建UTM的EPSG代码
    utm_epsg = f'EPSG:{32600 + zone if hemisphere == "N" else 32700 + zone}'
    wgs84 = 'EPSG:4326'
    
    # 创建转换器
    transformer = Transformer.from_crs(utm_epsg, wgs84, always_xy=True)
    
    # 转换坐标 (东向, 北向) -> (经度, 纬度)
    longitude, latitude = transformer.transform(utm_x, utm_y)
    
    return longitude, latitude


# ==================== 公共接口函数 ====================

def calculate_3dbox_projection_ratio(
    box_center,
    box_size,
    box_rotation=None,
    camera_translation=None,
    camera_rotation=None,
    camera_intrinsics=None,
    background_image=None,
    output_image=None,
    image_width=1920,
    image_height=1080
):
    """
    计算3D box投影在相机视野内的面积占比（公共接口）
    
    这是一个简洁的接口函数，用于计算3D box在相机图像中的投影区域占比。
    使用外接矩形面积来代表投影区域的大小。
    
    Args:
        box_center: 3D box在map坐标系下的中心点，array-like (x, y, z)
        box_size: 3D box的尺寸，array-like (length, width, height)
        box_rotation: 3D box的旋转，可以是：
            - 3x3 numpy array (旋转矩阵)
            - None (无旋转)
        camera_translation: 相机在map坐标系下的位置，array-like (x, y, z)
        camera_rotation: 相机的旋转，可以是：
            - 3x3 numpy array (旋转矩阵)
            - (w, x, y, z) 四元数
            - None (单位矩阵)
        camera_intrinsics: 相机内参，可以是：
            - dict: {'fx': float, 'fy': float, 'cx': float, 'cy': float}
            - dict: {'K': 3x3 numpy array}
            - 3x3 numpy array (内参矩阵)
        image_width: 图像宽度（像素），默认1920
        image_height: 图像高度（像素），默认1080
    
    Returns:
        ratio: float, 3D box投影外接矩形占图像总面积的比例 [0.0, 1.0]
               如果box不在相机视野内，返回 0.0
    
    Example:
        >>> ratio = calculate_3dbox_projection_ratio(
        ...     box_center=[10.0, 0.0, 0.0],
        ...     box_size=[4.5, 2.0, 1.8],
        ...     box_rotation=np.eye(3),
        ...     camera_translation=[0.0, 0.0, 1.5],
        ...     camera_rotation=np.eye(3),
        ...     camera_intrinsics={'fx': 1000, 'fy': 1000, 'cx': 960, 'cy': 540},
        ...     image_width=1920,
        ...     image_height=1080
        ... )
        >>> print(f"投影占比: {ratio:.2%}")
    """
    # 参数处理和验证
    box_center = np.asarray(box_center, dtype=np.float64)
    box_size = np.asarray(box_size, dtype=np.float64)
    box_size =  box_size * 2
    
    if box_rotation is None:
        box_rotation = np.eye(3)
    else:
        box_rotation = np.asarray(box_rotation, dtype=np.float64)
    
    if camera_translation is None:
        camera_translation = np.zeros(3)
    else:
        camera_translation = np.asarray(camera_translation, dtype=np.float64)
    
    # 处理相机旋转
    camera_rot_matrix = None
    camera_rot_quat = None
    
    if camera_rotation is None:
        camera_rot_matrix = np.eye(3)
    else:
        camera_rotation = np.asarray(camera_rotation, dtype=np.float64)
        if camera_rotation.shape == (3, 3):
            camera_rot_matrix = camera_rotation
        elif camera_rotation.shape == (4,):
            camera_rot_quat = camera_rotation
        else:
            raise ValueError(f"Invalid camera_rotation shape: {camera_rotation.shape}, expected (3,3) or (4,)")
    
    # 构建相机pose字典
    camera_pose_map = {
        'translation': camera_translation
    }
    if camera_rot_matrix is not None:
        camera_pose_map['rotation_matrix'] = camera_rot_matrix
    if camera_rot_quat is not None:
        camera_pose_map['rotation_quat'] = camera_rot_quat
    
    # 处理相机内参
    if camera_intrinsics is None:
        # 使用默认内参
        intrinsics_dict = {
            'fx': 1000.0,
            'fy': 1000.0,
            'cx': image_width / 2.0,
            'cy': image_height / 2.0
        }
    elif isinstance(camera_intrinsics, dict):
        intrinsics_dict = camera_intrinsics
    elif isinstance(camera_intrinsics, np.ndarray) and camera_intrinsics.shape == (3, 3):
        intrinsics_dict = {'K': camera_intrinsics}
    else:
        raise ValueError("camera_intrinsics must be dict or 3x3 numpy array")
    # 调用内部函数进行投影计算
    # result = project_3dbox_to_camera(
    result = project_3dbox_to_camera_2d(
        box_center_map=box_center,
        box_size_lwh=box_size,
        box_rotation_matrix=box_rotation,
        camera_pose_map=camera_pose_map,
        camera_intrinsics=intrinsics_dict,
        image_width=image_width,
        image_height=image_height,
        output_image_path=output_image,
        background_image=background_image,
        calculate_pixel_ratio=True
    )
    
    # 返回比例
    return result.get('pixel_ratio', -1.0)


# ==================== 内部辅助函数 ====================

def get_3dbox_corners(center, size_lwh, rotation_matrix=None):
    """
    获取3D box的8个角点坐标
    
    Args:
        center: 3D box中心点 (x, y, z)
        size_lwh: 3D box尺寸 (length, width, height)
        rotation_matrix: 3x3旋转矩阵，如果为None则表示无旋转
    
    Returns:
        corners: (8, 3) array of 3D corners
    """
    l, w, h = size_lwh
    # 定义局部坐标系下的8个角点（中心在原点）
    # 按照标准顺序：前面4个点（底部），后面4个点（顶部）
    x_corners = np.array([l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2])
    y_corners = np.array([w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2])
    z_corners = np.array([-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2])
    
    corners = np.vstack([x_corners, y_corners, z_corners])  # (3, 8)
    
    # 应用旋转
    if rotation_matrix is not None:
        corners = rotation_matrix @ corners  # (3, 8)
    
    # 平移到中心点
    corners = corners.T + np.array(center)  # (8, 3)
    
    return corners


def transform_pose_to_matrix(translation, rotation_quat=None, rotation_matrix=None):
    """
    将pose转换为4x4变换矩阵
    
    Args:
        translation: (x, y, z) 平移向量
        rotation_quat: (w, x, y, z) 四元数，或
        rotation_matrix: 3x3旋转矩阵
    
    Returns:
        transform: 4x4变换矩阵
    """
    T = np.eye(4)
    T[:3, 3] = translation
    
    if rotation_matrix is not None:
        T[:3, :3] = rotation_matrix
    elif rotation_quat is not None:
        # 四元数转旋转矩阵
        w, x, y, z = rotation_quat
        T[:3, :3] = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
    
    return T


def get_box_2d_polygon(corners_2d, visibility):
    """
    根据3D box的8个角点投影和可见性，构建2D多边形
    
    Args:
        corners_2d: (8, 2) 投影后的2D角点坐标
        visibility: (8,) bool数组，表示每个角点是否在相机前方
    
    Returns:
        polygon_points: 多边形顶点列表（按顺序）
    """
    # 3D box的面定义（每个面由4个角点索引组成）
    # 角点索引：底面 0-3，顶面 4-7
    faces = [
        [0, 1, 2, 3],  # 底面
        [4, 5, 6, 7],  # 顶面
        [0, 1, 5, 4],  # 前面
        [2, 3, 7, 6],  # 后面
        [0, 3, 7, 4],  # 左面
        [1, 2, 6, 5],  # 右面
    ]
    
    # 收集所有可能可见的面的边
    edges_set = set()
    
    for face in faces:
        # 检查这个面是否至少有一个角点可见（在相机前方）
        face_has_visible = any(visibility[i] for i in face)
        if face_has_visible:
            # 添加这个面的所有边
            for i in range(4):
                p1 = face[i]
                p2 = face[(i + 1) % 4]
                # 保持边的方向一致性
                edge = tuple(sorted([p1, p2]))
                edges_set.add(edge)
    
    # 构建多边形顶点（使用所有相关的角点）
    vertices_set = set()
    for edge in edges_set:
        vertices_set.add(edge[0])
        vertices_set.add(edge[1])
    
    if len(vertices_set) < 3:
        return np.array([])
    
    # 获取这些顶点的2D坐标
    vertices_list = list(vertices_set)
    polygon_points = corners_2d[vertices_list]
    
    # 计算中心点
    center = np.mean(polygon_points, axis=0)
    
    # 按极坐标角度排序顶点
    angles = np.arctan2(polygon_points[:, 1] - center[1], 
                       polygon_points[:, 0] - center[0])
    sorted_indices = np.argsort(angles)
    polygon_points = polygon_points[sorted_indices]
    
    return polygon_points


def clip_polygon_to_image(polygon_points, image_width, image_height):
    """
    将多边形裁剪到图像边界内
    
    Args:
        polygon_points: (N, 2) 多边形顶点坐标
        image_width: 图像宽度
        image_height: 图像高度
    
    Returns:
        clipped_polygon: 裁剪后的多边形顶点
    """
    if len(polygon_points) == 0:
        return np.array([])
    
    # 使用Sutherland-Hodgman算法裁剪多边形
    # 定义图像边界
    clip_edges = [
        ([0, 0], [image_width, 0]),           # 上边界
        ([image_width, 0], [image_width, image_height]),  # 右边界
        ([image_width, image_height], [0, image_height]), # 下边界
        ([0, image_height], [0, 0])            # 左边界
    ]
    
    def inside_edge(point, edge_start, edge_end):
        """判断点是否在边的内侧（左侧）"""
        return (edge_end[0] - edge_start[0]) * (point[1] - edge_start[1]) - \
               (edge_end[1] - edge_start[1]) * (point[0] - edge_start[0]) >= 0
    
    def line_intersection(p1, p2, edge_start, edge_end):
        """计算线段与边的交点"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = edge_start
        x4, y4 = edge_end
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        
        intersection = [x1 + t * (x2 - x1), y1 + t * (y2 - y1)]
        return intersection
    
    # 对每条裁剪边进行裁剪
    output = polygon_points.tolist()
    
    for edge_start, edge_end in clip_edges:
        if len(output) == 0:
            break
        
        input_list = output
        output = []
        
        for i in range(len(input_list)):
            current = input_list[i]
            previous = input_list[i - 1]
            
            current_inside = inside_edge(current, edge_start, edge_end)
            previous_inside = inside_edge(previous, edge_start, edge_end)
            
            if current_inside:
                if not previous_inside:
                    # 从外到内，添加交点
                    intersection = line_intersection(previous, current, edge_start, edge_end)
                    if intersection is not None:
                        output.append(intersection)
                output.append(current)
            elif previous_inside:
                # 从内到外，添加交点
                intersection = line_intersection(previous, current, edge_start, edge_end)
                if intersection is not None:
                    output.append(intersection)
    
    return np.array(output) if len(output) > 0 else np.array([])


def calculate_box_pixel_ratio(corners_2d, visibility, image_width, image_height):
    """
    计算3D box投影区域在图像中的像素占比
    
        该函数计算 3D box 投影外接矩形与相机图像矩形的重叠面积占比。
    
        说明：
        - 传统“只考虑落在图像内的角点/多边形裁剪”的方法，在 box 很大、
            但所有角点都投影到图像外时，会误判为 0。
        - 这里改为：直接计算投影外接矩形（仅使用相机前方的角点）与图像矩形
            的交集面积 / 图像面积，不依赖外接矩形四个角是否在图像内。
    
    Args:
        corners_2d: (8, 2) 投影后的2D角点坐标
                visibility: (8,) bool数组，表示每个角点是否有效（通常应为“在相机前方”）
        image_width: 图像宽度
        image_height: 图像高度
    
    Returns:
        pixel_ratio: 框内像素占总像素的比例
        pixel_count: 框内的像素数量
        total_pixels: 图像总像素数
        bounding_rect: 外接矩形 (x_min, y_min, x_max, y_max)，如果无效则为None
    """
    total_pixels = image_width * image_height
    
    # 如果没有任何有效角点（例如全部在相机后方），返回0
    valid_mask = np.asarray(visibility, dtype=bool)
    if not np.any(valid_mask):
        return 0.0, 0, total_pixels, None

    valid_pts = np.asarray(corners_2d, dtype=np.float64)[valid_mask]
    if valid_pts.shape[0] < 1:
        return 0.0, 0, total_pixels, None

    # 投影外接矩形（不裁剪到图像内）
    proj_x_min = float(np.min(valid_pts[:, 0]))
    proj_y_min = float(np.min(valid_pts[:, 1]))
    proj_x_max = float(np.max(valid_pts[:, 0]))
    proj_y_max = float(np.max(valid_pts[:, 1]))

    # 和图像矩形 [0, W) x [0, H) 求交集
    inter_x_min = max(0.0, proj_x_min)
    inter_y_min = max(0.0, proj_y_min)
    inter_x_max = min(float(image_width), proj_x_max)
    inter_y_max = min(float(image_height), proj_y_max)

    inter_w = inter_x_max - inter_x_min
    inter_h = inter_y_max - inter_y_min
    if inter_w <= 0.0 or inter_h <= 0.0:
        return 0.0, 0, total_pixels, None

    pixel_count = int(inter_w * inter_h)
    pixel_ratio = pixel_count / total_pixels
    bounding_rect = (inter_x_min, inter_y_min, inter_x_max, inter_y_max)
    return pixel_ratio, pixel_count, total_pixels, bounding_rect


def project_3dbox_to_camera(
    box_center_map,
    box_size_lwh,
    box_rotation_matrix,
    camera_pose_map,
    camera_intrinsics,
    image_width,
    image_height,
    output_image_path=None,
    background_image=None,
    box_color=(0, 255, 0),
    line_thickness=2,
    calculate_pixel_ratio=True
):
    """
    将3D box从map坐标系投影到相机图像平面
    
    Args:
        box_center_map: 3D box在map坐标系下的中心点 (x, y, z)
        box_size_lwh: 3D box的长宽高 (length, width, height)
        box_rotation_matrix: 3D box在map坐标系下的3x3旋转矩阵
        camera_pose_map: dict包含相机在map坐标系下的pose:
            - 'translation': (x, y, z) 相机位置
            - 'rotation_matrix': 3x3旋转矩阵 或
            - 'rotation_quat': (w, x, y, z) 四元数
        camera_intrinsics: dict包含相机内参:
            - 'fx': x方向焦距
            - 'fy': y方向焦距
            - 'cx': 主点x坐标
            - 'cy': 主点y坐标
            或者
            - 'K': 3x3相机内参矩阵
        image_width: 图像宽度（像素）
        image_height: 图像高度（像素）
        output_image_path: 输出图像路径（可选）
        background_image: 背景图像（可选），如果提供则在该图像上绘制
        box_color: 边界框颜色 (B, G, R)，默认绿色
        line_thickness: 线条粗细
        calculate_pixel_ratio: 是否计算像素占比，默认True
    
    Returns:
        result: dict包含以下键:
            - 'image': 绘制了3D box投影的图像（如果提供了背景图像或输出路径）
            - 'corners_2d': (8, 2) 投影后的2D角点坐标
            - 'visibility': (8,) bool数组，表示每个角点是否在相机前方且在图像内
            - 'pixel_ratio': 框内像素占总像素的比例（如果calculate_pixel_ratio=True）
            - 'pixel_count': 框内的像素数量（如果calculate_pixel_ratio=True）
            - 'total_pixels': 图像总像素数（如果calculate_pixel_ratio=True）
    """
    # 1. 获取3D box的8个角点（在map坐标系下）
    corners_3d_map = get_3dbox_corners(box_center_map, box_size_lwh, box_rotation_matrix)
    
    # 2. 构建相机的变换矩阵（map -> camera）
    camera_trans = camera_pose_map['translation']
    camera_rot_matrix = camera_pose_map.get('rotation_matrix')
    camera_rot_quat = camera_pose_map.get('rotation_quat')
    
    # 相机在map坐标系下的变换矩阵
    T_map_to_cam_world = transform_pose_to_matrix(camera_trans, camera_rot_quat, camera_rot_matrix)
    
    # 需要从world到camera的逆变换
    T_world_to_cam = np.linalg.inv(T_map_to_cam_world)
    
    # 3. 将3D角点从map坐标系转换到相机坐标系
    corners_3d_map_homo = np.hstack([corners_3d_map, np.ones((8, 1))])  # (8, 4)
    corners_3d_cam_homo = (T_world_to_cam @ corners_3d_map_homo.T).T  # (8, 4)
    corners_3d_cam = corners_3d_cam_homo[:, :3]  # (8, 3)
    
    # 4. 准备相机内参矩阵
    if 'K' in camera_intrinsics:
        K = camera_intrinsics['K']
    else:
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
    
    # 5. 投影到图像平面
    corners_2d = []
    visibility = []
    in_front = []
    
    for corner_3d in corners_3d_cam:
        # 检查点是否在相机前方
        if corner_3d[2] <= 0:
            corners_2d.append([np.nan, np.nan])
            visibility.append(False)
            in_front.append(False)
            continue
        in_front.append(True)
        
        # 透视投影
        corner_2d_homo = K @ corner_3d
        corner_2d = corner_2d_homo[:2] / corner_2d_homo[2]
        
        # 检查是否在图像范围内
        x, y = corner_2d
        in_image = (0 <= x < image_width) and (0 <= y < image_height)
        
        corners_2d.append(corner_2d)
        visibility.append(in_image and corner_3d[2] > 0)
    
    corners_2d = np.array(corners_2d)  # (8, 2)
    visibility = np.array(visibility)  # (8,)
    in_front = np.array(in_front, dtype=bool)  # (8,)
    
    # 6. 计算像素占比（如果需要）
    pixel_ratio = None
    pixel_count = None
    total_pixels = image_width * image_height
    bounding_rect = None
    
    if calculate_pixel_ratio:
        pixel_ratio, pixel_count, total_pixels, bounding_rect = calculate_box_pixel_ratio(
            corners_2d, in_front, image_width, image_height
        )
    
    # 7. 绘制3D box（如果需要）
    image = None
    if output_image_path is not None or background_image is not None:
        # 基于“相机视野矩形”和“投影外接矩形”生成新的画布，不裁剪到原图范围。
        valid_mask = np.asarray(in_front, dtype=bool) & np.isfinite(corners_2d[:, 0]) & np.isfinite(corners_2d[:, 1])
        if np.any(valid_mask):
            valid_pts = corners_2d[valid_mask]
            proj_x_min = float(np.min(valid_pts[:, 0]))
            proj_y_min = float(np.min(valid_pts[:, 1]))
            proj_x_max = float(np.max(valid_pts[:, 0]))
            proj_y_max = float(np.max(valid_pts[:, 1]))

            # 与相机视野矩形 [0,W]x[0,H] 判断是否相交（包含相机视野也属于相交）
            inter_x_min = max(0.0, proj_x_min)
            inter_y_min = max(0.0, proj_y_min)
            inter_x_max = min(float(image_width), proj_x_max)
            inter_y_max = min(float(image_height), proj_y_max)
            intersects = (inter_x_max > inter_x_min) and (inter_y_max > inter_y_min)

            if intersects:
                margin = 20.0
                canvas_x_min = min(0.0, proj_x_min) - margin
                canvas_y_min = min(0.0, proj_y_min) - margin
                canvas_x_max = max(float(image_width), proj_x_max) + margin
                canvas_y_max = max(float(image_height), proj_y_max) + margin

                canvas_w = max(1.0, canvas_x_max - canvas_x_min)
                canvas_h = max(1.0, canvas_y_max - canvas_y_min)

                # 防止投影特别大导致图像爆内存：自动缩放到 max_dim
                max_dim = 4000.0
                scale = 1.0
                max_side = max(canvas_w, canvas_h)
                if max_side > max_dim:
                    scale = max_dim / max_side

                out_w = int(np.ceil(canvas_w * scale))
                out_h = int(np.ceil(canvas_h * scale))
                image = np.ones((out_h, out_w, 3), dtype=np.uint8) * 255

                def map_pt(xy):
                    x, y = float(xy[0]), float(xy[1])
                    u = int(np.round((x - canvas_x_min) * scale))
                    v = int(np.round((y - canvas_y_min) * scale))
                    return (u, v)

                # 可选：把背景图贴到相机视野区域
                if background_image is not None:
                    bg = None
                    if isinstance(background_image, str):
                        bg = cv2.imread(background_image)
                    else:
                        bg = background_image.copy()

                    if bg is not None:
                        # 将背景图缩放到相机视野大小
                        if bg.shape[1] != image_width or bg.shape[0] != image_height:
                            bg = cv2.resize(bg, (int(image_width), int(image_height)))

                        bg_scaled = bg
                        if scale != 1.0:
                            bg_scaled = cv2.resize(bg, (int(np.round(image_width * scale)), int(np.round(image_height * scale))))

                        cam_tl = map_pt((0.0, 0.0))
                        x0, y0 = cam_tl
                        x1 = min(out_w, x0 + bg_scaled.shape[1])
                        y1 = min(out_h, y0 + bg_scaled.shape[0])
                        if x0 < out_w and y0 < out_h and x1 > 0 and y1 > 0:
                            image[y0:y1, x0:x1] = bg_scaled[0:(y1 - y0), 0:(x1 - x0)]

                # 画相机视野矩形（蓝色）
                cam_pt1 = map_pt((0.0, 0.0))
                cam_pt2 = map_pt((float(image_width), float(image_height)))
                cv2.rectangle(image, cam_pt1, cam_pt2, (255, 0, 0), max(1, line_thickness))

                # 画投影外接矩形（红色）
                bbox_pt1 = map_pt((proj_x_min, proj_y_min))
                bbox_pt2 = map_pt((proj_x_max, proj_y_max))
                cv2.rectangle(image, bbox_pt1, bbox_pt2, (0, 0, 255), max(1, line_thickness))

                # 画3D box投影线框（不裁剪、不要求点在图像内，只要求在相机前方）
                edges = [
                    (0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7)
                ]
                for i, j in edges:
                    if valid_mask[i] and valid_mask[j]:
                        pt1 = map_pt(corners_2d[i])
                        pt2 = map_pt(corners_2d[j])
                        cv2.line(image, pt1, pt2, box_color, max(1, line_thickness))

                # 在图像中心绘制 pixel_ratio 文本，便于调试
                if pixel_ratio is not None:
                    text = f"pixel_ratio: {pixel_ratio:.6f}"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = max(0.6, min(out_w, out_h) / 1200.0)
                    thickness = max(1, int(np.ceil(2 * font_scale)))
                    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
                    cx = out_w // 2
                    cy = out_h // 2
                    org = (int(cx - tw / 2), int(cy + th / 2))
                    pad = int(max(6, 6 * font_scale))
                    rect_pt1 = (org[0] - pad, org[1] - th - pad)
                    rect_pt2 = (org[0] + tw + pad, org[1] + baseline + pad)
                    cv2.rectangle(image, rect_pt1, rect_pt2, (255, 255, 255), -1)
                    cv2.putText(image, text, org, font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

                # 保存 debug 图（只要相交就保存）
                if output_image_path is not None:
                    cv2.imwrite(output_image_path, image)
                    print(f"[OK] Saved debug projection image to: {output_image_path}")
    
    # 8. 返回结果字典
    result = {
        'image': image,
        'corners_2d': corners_2d,
        'visibility': visibility
    }
    
    if calculate_pixel_ratio:
        result['pixel_ratio'] = pixel_ratio
        result['pixel_count'] = pixel_count
        result['total_pixels'] = total_pixels
        result['bounding_rect'] = bounding_rect
    
    return result


def project_3dbox_to_camera_2d(
    box_center_map,
    box_size_lwh,
    box_rotation_matrix,
    camera_pose_map,
    camera_intrinsics,
    image_width,
    image_height,
    output_image_path=None,
    background_image=None,
    box_color=(0, 255, 0),
    line_thickness=2,
    calculate_pixel_ratio=True
):
    """ 
    2D版本：基于相机水平FOV在水平面(XY)上的覆盖区域，估算其与3D box在水平面投影矩形的覆盖比。

    逻辑：
    - 由相机内参计算水平FOV：$fov_x = 2 * atan(W / (2 * fx))$
    - 使用相机朝向（旋转矩阵第三列在XY平面的投影）作为中心方向
    - 在XY平面构造一个FOV三角形（camera位置 + 左/右边界射线延伸到足够远的点）
    - 计算该三角形与 box 在XY平面的轴对齐外接矩形的交集面积
    - pixel_ratio = intersection_area / box_rect_area

    返回字段尽量保持与 `project_3dbox_to_camera` 相似：
    - 'image': 可视化图（如果保存）
    - 'pixel_ratio', 'pixel_count', 'total_pixels'
    额外包含：'box_rect', 'fov_polygon', 'intersection_poly', 'intersection_area', 'box_area'

    注意：这里的 pixel_ratio 是“地面面积比”，不是严格的像素比，但接口形态一致便于上层复用。
    """
    box_center_map = np.asarray(box_center_map, dtype=np.float64)

    # box在XY平面的轴对齐外接矩形
    corners_3d_map = get_3dbox_corners(box_center_map, box_size_lwh, box_rotation_matrix)
    xs = corners_3d_map[:, 0]
    ys = corners_3d_map[:, 1]
    x_min, x_max = float(np.min(xs)), float(np.max(xs))
    y_min, y_max = float(np.min(ys)), float(np.max(ys))
    box_rect = np.array([
        [x_min, y_min],
        [x_max, y_min],
        [x_max, y_max],
        [x_min, y_max]
    ], dtype=np.float64)

    box_area = float(max(0.0, (x_max - x_min) * (y_max - y_min)))

    # 相机位姿
    cam_trans = np.asarray(camera_pose_map['translation'], dtype=np.float64)
    cam_rot = camera_pose_map.get('rotation_matrix')
    cam_quat = camera_pose_map.get('rotation_quat')
    if cam_rot is None and cam_quat is not None:
        q = np.asarray(cam_quat, dtype=np.float64)
        if q.shape != (4,):
            raise ValueError(f"Invalid rotation_quat shape: {q.shape}, expected (4,)")
        # q is [w,x,y,z] -> scipy expects [x,y,z,w]
        cam_rot = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    if cam_rot is None:
        cam_rot = np.eye(3, dtype=np.float64)

    cam_xy = cam_trans[:2].astype(np.float64)

    # 相机到 box_center_map 的距离（3D）
    cam_to_box_dist_m = float(np.linalg.norm(cam_trans.reshape(3,) - box_center_map.reshape(3,)))

    # 相机水平FOV（从内参取 fx）
    if 'K' in camera_intrinsics:
        K = np.asarray(camera_intrinsics['K'], dtype=np.float64)
        fx = float(K[0, 0])
    else:
        fx = float(camera_intrinsics.get('fx', 1.0))
    if not np.isfinite(fx) or fx <= 0:
        fx = 1.0
    fov_x = 2.0 * float(np.arctan(float(image_width) / (2.0 * fx)))

    # 相机前向方向（取旋转矩阵第三列，投影到XY）
    forward = np.asarray(cam_rot[:, 2], dtype=np.float64)
    f_xy = forward[:2]
    n = float(np.linalg.norm(f_xy))
    if n < 1e-9:
        f_xy = np.array([1.0, 0.0], dtype=np.float64)
    else:
        f_xy = f_xy / n

    center_angle = float(np.arctan2(f_xy[1], f_xy[0]))
    left_angle = center_angle + fov_x / 2.0
    right_angle = center_angle - fov_x / 2.0

    # 半径选取：保证能覆盖到box（取相机到box四角最大距离的2倍）
    dists = np.linalg.norm(box_rect - cam_xy.reshape(1, 2), axis=1)
    max_dist = float(np.max(dists)) if dists.size else 1.0
    radius = 400 # max(1.0, max_dist * 2.0)

    left_far = cam_xy + radius * np.array([np.cos(left_angle), np.sin(left_angle)], dtype=np.float64)
    right_far = cam_xy + radius * np.array([np.cos(right_angle), np.sin(right_angle)], dtype=np.float64)

    # FOV三角形（不要求严格扇形，使用三角形近似）
    fov_polygon = np.vstack([cam_xy, left_far, right_far]).astype(np.float64)

    def polygon_area(poly: np.ndarray) -> float:
        if poly is None or len(poly) < 3:
            return 0.0
        x = poly[:, 0]
        y = poly[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    # Sutherland–Hodgman：裁剪 subject(三角形) 到 clipper(矩形)
    def _is_inside(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> bool:
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -1e-12

    def _line_intersection(p1: np.ndarray, p2: np.ndarray, a: np.ndarray, b: np.ndarray):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = a
        x4, y4 = b
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-12:
            return None
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        return np.array([x1 + t * (x2 - x1), y1 + t * (y2 - y1)], dtype=np.float64)

    def polygon_clip(subject: np.ndarray, clipper: np.ndarray) -> np.ndarray:
        output = subject.tolist()
        clip = clipper.tolist()
        for i in range(len(clip)):
            input_list = output
            output = []
            if len(input_list) == 0:
                break
            A = np.array(clip[i - 1], dtype=np.float64)
            B = np.array(clip[i], dtype=np.float64)
            for j in range(len(input_list)):
                P = np.array(input_list[j], dtype=np.float64)
                Q = np.array(input_list[j - 1], dtype=np.float64)
                P_in = _is_inside(P, A, B)
                Q_in = _is_inside(Q, A, B)
                if P_in:
                    if not Q_in:
                        inter = _line_intersection(Q, P, A, B)
                        if inter is not None:
                            output.append(inter.tolist())
                    output.append(P.tolist())
                elif Q_in:
                    inter = _line_intersection(Q, P, A, B)
                    if inter is not None:
                        output.append(inter.tolist())
        return np.array(output, dtype=np.float64) if len(output) > 0 else np.array([], dtype=np.float64)

    intersection_poly = polygon_clip(fov_polygon, box_rect) if box_area > 0 else np.array([], dtype=np.float64)
    intersection_area = float(polygon_area(intersection_poly)) if intersection_poly.size else 0.0

    total_pixels = int(image_width * image_height)
    pixel_ratio = None
    pixel_count = None

    if calculate_pixel_ratio and box_area > 0:
        pixel_ratio = float(intersection_area) / float(box_area)
        # 数值稳定：裁剪到[0,1]
        pixel_ratio = float(np.clip(pixel_ratio, 0.0, 1.0))
        pixel_count = int(round(pixel_ratio * total_pixels)) if total_pixels > 0 else 0
    elif calculate_pixel_ratio:
        pixel_ratio = 0.0
        pixel_count = 0

    # 可视化（保存两个区域 + 交集）
    vis_image = None
    if output_image_path is not None:
        # 固定画布大小与显示范围：以 box_center_map 为中心，上下左右各 300m
        canvas_size = 900
        Wc = canvas_size
        Hc = canvas_size

        box_center_xy = box_center_map[:2].astype(np.float64)
        half_range_m = 300.0
        min_xy = box_center_xy - half_range_m
        max_xy = box_center_xy + half_range_m

        dx = float(max_xy[0] - min_xy[0])
        dy = float(max_xy[1] - min_xy[1])
        if dx <= 1e-9:
            dx = 1.0
        if dy <= 1e-9:
            dy = 1.0
        # 20px 边距：坐标系稳定
        scale = min((canvas_size - 40) / dx, (canvas_size - 40) / dy)
        canvas = np.ones((Hc, Wc, 3), dtype=np.uint8) * 255

        def world2pix(pt):
            x, y = float(pt[0]), float(pt[1])
            u = int(round((x - min_xy[0]) * scale)) + 20
            v = int(round((max_xy[1] - y) * scale)) + 20
            return (u, v)

        # box rect（浅蓝填充 + 黑边）
        box_pts_pix = np.array([world2pix(p) for p in box_rect], dtype=np.int32)
        cv2.fillPoly(canvas, [box_pts_pix], color=(200, 230, 255))
        cv2.polylines(canvas, [box_pts_pix], isClosed=True, color=(0, 0, 0), thickness=max(1, int(line_thickness)))

        # fov polygon（浅绿填充 + 绿边）
        fov_pts_pix = np.array([world2pix(p) for p in fov_polygon], dtype=np.int32)
        cv2.fillPoly(canvas, [fov_pts_pix], color=(220, 255, 220))
        cv2.polylines(canvas, [fov_pts_pix], isClosed=True, color=(0, 128, 0), thickness=max(1, int(line_thickness)))

        # intersection（浅红填充 + 红边）
        if intersection_poly.size:
            inter_pts_pix = np.array([world2pix(p) for p in intersection_poly], dtype=np.int32)
            cv2.fillPoly(canvas, [inter_pts_pix], color=(255, 200, 200))
            cv2.polylines(canvas, [inter_pts_pix], isClosed=True, color=(0, 0, 255), thickness=max(1, int(line_thickness)))

        # camera 点
        cam_px = world2pix(cam_xy)
        cv2.circle(canvas, cam_px, 6, (0, 0, 0), -1)
        cv2.putText(canvas, 'Camera', (cam_px[0] + 8, cam_px[1] + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # box center 点 + 距离文本（显示在 box_center_map 位置附近）
        box_center_px = world2pix(box_center_xy)
        cv2.drawMarker(canvas, box_center_px, (0, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
        dist_text = f"{cam_to_box_dist_m:.2f}m"
        text_org = (box_center_px[0] + 8, box_center_px[1] - 8)
        # 简单边界保护，避免文字完全跑出画布
        text_org = (int(np.clip(text_org[0], 0, Wc - 1)), int(np.clip(text_org[1], 0, Hc - 1)))
        cv2.putText(canvas, dist_text, text_org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # 文本信息
        if pixel_ratio is not None:
            box_size_lwh_arr = np.asarray(box_size_lwh, dtype=np.float64).reshape(-1)
            box_size_lwh_str = np.array2string(box_size_lwh_arr, precision=1, separator=',')
            info_line1 = f"box_size_lwh={box_size_lwh_str}, box_area={box_area:.3f}, inter={intersection_area:.3f}"
            info_line2 = f"ratio={pixel_ratio:.6f}, fov_x(deg)={np.degrees(fov_x):.2f}, cam_dist={cam_to_box_dist_m:.2f}m"

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            line_gap = 4
            (_, th), _ = cv2.getTextSize("Ag", font, font_scale, thickness)
            y2 = Hc - 10
            y1 = y2 - th - line_gap
            cv2.putText(canvas, info_line1, (10, y1), font, font_scale, (0, 0, 0), thickness)
            cv2.putText(canvas, info_line2, (10, y2), font, font_scale, (0, 0, 0), thickness)

        # 将 background_image 贴到左上角（作为参考）
        if background_image is not None:
            bg = None
            if isinstance(background_image, str):
                bg = cv2.imread(background_image)
            else:
                # assume ndarray
                try:
                    bg = background_image.copy()
                except Exception:
                    bg = None

            if bg is not None and bg.size != 0:
                # 统一为BGR 3通道
                if bg.ndim == 2:
                    bg = cv2.cvtColor(bg, cv2.COLOR_GRAY2BGR)
                elif bg.ndim == 3 and bg.shape[2] == 4:
                    bg = cv2.cvtColor(bg, cv2.COLOR_BGRA2BGR)

                # 缩放到画布的一个角落大小
                pad_px = 10
                max_w = int(Wc * 0.35)
                max_h = int(Hc * 0.35)
                if max_w > 10 and max_h > 10:
                    scale_bg = min(max_w / bg.shape[1], max_h / bg.shape[0], 1.0)
                    new_w = max(1, int(round(bg.shape[1] * scale_bg)))
                    new_h = max(1, int(round(bg.shape[0] * scale_bg)))
                    bg_small = cv2.resize(bg, (new_w, new_h), interpolation=cv2.INTER_AREA)

                    # 左上角区域坐标
                    x0 = pad_px
                    y0 = pad_px
                    x1 = min(Wc, x0 + new_w)
                    y1 = min(Hc, y0 + new_h)
                    # 可能因边界裁剪导致尺寸不一致，做安全切片
                    paste_w = max(0, x1 - x0)
                    paste_h = max(0, y1 - y0)
                    if paste_w > 0 and paste_h > 0:
                        canvas[y0:y1, x0:x1] = bg_small[0:paste_h, 0:paste_w]

                        # 画一个边框
                        cv2.rectangle(canvas, (x0, y0), (x1, y1), (50, 50, 50), 1)
                        cv2.putText(canvas, 'BG', (x0 + 5, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 50, 50), 2)

        vis_image = canvas

        base, ext = os.path.splitext(output_image_path)
        save_path = f"{base}_fov_box.png" if base else (output_image_path + "_fov_box.png")
        cv2.imwrite(save_path, vis_image)

    result2d = {
        'image': vis_image,
        'box_rect': box_rect,
        'fov_polygon': fov_polygon,
        'intersection_poly': intersection_poly,
        'intersection_area': intersection_area,
        'box_area': box_area,
        'cam_to_box_dist_m': cam_to_box_dist_m,
        'pixel_ratio': pixel_ratio,
        'pixel_count': pixel_count,
        'total_pixels': total_pixels,
    }

    return result2d

def load_lidar_pose_from_file(pose_file_path):
    """
    从文件加载雷达在map坐标系下的pose
    
    文件格式示例：
    timestamp: 1763621136.500
    pose_utm:
      - [4x4 transformation matrix elements, row-major order, 16 values]
    offset_utm:
      - [x_offset, y_offset]
    
    Args:
        pose_file_path: pose文件路径
    
    Returns:
        pose: dict包含:
            - 'translation': (x, y, z) 平移向量
            - 'rotation_matrix': 3x3旋转矩阵
            - 'quaternion': (w, x, y, z) 四元数
            - 'timestamp': 时间戳
            - 'pose_utm': 4x4变换矩阵
            - 'offset_utm': UTM偏移 (x, y)
        如果加载失败返回None
    """
    try:
        # 检查文件是否存在
        if not os.path.exists(pose_file_path):
            print(f"[ERROR] Pose文件不存在: {pose_file_path}")
            return None
        
        # 使用yaml加载文件
        with open(pose_file_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 提取时间戳
        timestamp = data.get('timestamp', 0.0)
        
        # 提取pose_utm（4x4变换矩阵，存储为16个元素的列表）
        pose_utm_list = data.get('pose_utm', [])
        if len(pose_utm_list) != 16:
            print(f"[ERROR] pose_utm格式错误，期望16个元素，实际{len(pose_utm_list)}个")
            return None
        
        # 将列表转换为4x4矩阵（row-major order）
        pose_utm_matrix = np.array(pose_utm_list).reshape(4, 4)
        
        # 提取offset_utm
        offset_utm = data.get('offset_utm', [0.0, 0.0])
        
        # 从4x4矩阵提取旋转和平移
        rotation_matrix = pose_utm_matrix[:3, :3]
        translation = pose_utm_matrix[:3, 3] + np.array([offset_utm[0], offset_utm[1], 0.0])
        
        # 旋转矩阵转四元数
        r = Rotation.from_matrix(rotation_matrix)
        quat_xyzw = r.as_quat()  # [x, y, z, w]
        quaternion = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # 转为 [w, x, y, z]
        
        pose = {
            'translation': translation,
            'rotation_matrix': rotation_matrix,
            'quaternion': quaternion,
            'timestamp': timestamp,
            'pose_utm': pose_utm_matrix,
            'offset_utm': np.array(offset_utm)
        }
        
        # print(f"[OK] 成功加载pose文件: {os.path.basename(pose_file_path)}")
        # print(f"  时间戳: {timestamp}")
        # print(f"  位置(UTM): {translation}")
        
        return pose
        
    except Exception as e:
        print(f"[ERROR] 加载pose文件失败: {pose_file_path}")
        print(f"  错误信息: {e}")
        return None


def plot_camera_positions_simple(box_center, box_size, cam_poses_in_map, save_path: str = None, point_size: int = 2):
    """
    简单绘制相机位置散点图，不考虑朝向
    
    Args:
        box_center: 3D box中心位置，array-like (x, y, z)
        box_size: 3D box尺寸，array-like (length, width, height)
        cam_poses_in_map: 可以是：
            - List[Dict]: 单个相机的pose列表，每个元素包含'filename'和'pose'
            - Dict[str, List[Dict]]: 多个相机的poses，key为相机名称，value为pose列表
        save_path: 保存图片的路径，如果为None则显示图片
        point_size: 散点的大小，默认2
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    
    fig, ax = plt.subplots(figsize=(15, 12))
    
    # 判断输入类型
    if isinstance(cam_poses_in_map, dict):
        # 多相机模式
        cameras_dict = cam_poses_in_map
        is_multi_camera = True
    else:
        # 单相机模式，包装成字典
        cameras_dict = {'camera': cam_poses_in_map}
        is_multi_camera = False
    
    # 定义7种不同的颜色用于区分不同相机
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan', 'magenta']
    
    # 遍历每个相机
    all_x = []
    all_y = []
    
    for cam_idx, (cam_name, cam_poses) in enumerate(cameras_dict.items()):
        color = colors[cam_idx % len(colors)]
        
        # 提取所有相机位置和朝向
        x_coords = []
        y_coords = []
        u_dirs = []  # x方向分量
        v_dirs = []  # y方向分量
        
        for cam_data in cam_poses:
            pose = cam_data['pose']
            pos = pose['translation']
            rot_mat = pose['rotation_matrix']
            
            x_coords.append(pos[0])
            y_coords.append(pos[1])
            all_x.append(pos[0])
            all_y.append(pos[1])
            
            # 获取z轴方向（旋转矩阵的第三列）
            z_axis = rot_mat[:, 2]
            u_dirs.append(z_axis[0])
            v_dirs.append(z_axis[1])
        
        # 绘制散点
        if is_multi_camera:
            ax.scatter(x_coords, y_coords, c=color, s=point_size, 
                      alpha=0.6, label=cam_name)
        else:
            ax.scatter(x_coords, y_coords, c=color, s=point_size, alpha=0.6)
            # 标记起点和终点
            if len(x_coords) > 0:
                ax.scatter(x_coords[0], y_coords[0], c='green', s=50, 
                          marker='o', label='Start', zorder=5)
                ax.scatter(x_coords[-1], y_coords[-1], c='red', s=50, 
                          marker='s', label='End', zorder=5)
        
        # 绘制朝向箭头（每隔N个画一个，避免太密集）
        # if len(x_coords) > 0:
        #     step = max(1, len(x_coords) // 10)  # 最多画50个箭头
        #     x_arrows = x_coords[::step]
        #     y_arrows = y_coords[::step]
        #     u_arrows = u_dirs[::step]
        #     v_arrows = v_dirs[::step]
            
            # 计算箭头长度（根据数据范围自适应）
            # if all_x and all_y:
            #     data_range = max(max(all_x) - min(all_x), max(all_y) - min(all_y))
            #     arrow_scale = data_range * 0.05  # 箭头长度为数据范围的2%
            # else:
            #     arrow_scale = 5.0
            
            # ax.quiver(x_arrows, y_arrows, u_arrows, v_arrows, 
            #          color=color, alpha=0.7, scale=1/arrow_scale, 
            #          scale_units='xy', width=0.003, headwidth=3, headlength=4)
            
            # 在箭头两侧各45度绘制虚线（长度为箭头长度的5倍）
            # for i in range(len(x_arrows)):
            #     # 获取当前箭头的方向向量
            #     u, v = u_arrows[i], v_arrows[i]
            #     # 计算箭头角度
            #     arrow_angle = np.arctan2(v, u)
                
            #     # 左侧虚线（+45度）
            #     left_angle = arrow_angle + np.pi / 4
            #     left_u = np.cos(left_angle) * 5 * arrow_scale
            #     left_v = np.sin(left_angle) * 5 * arrow_scale
            #     ax.plot([x_arrows[i], x_arrows[i] + left_u], 
            #            [y_arrows[i], y_arrows[i] + left_v], 
            #            color=color, alpha=0.5, linestyle='--', linewidth=1)
                
            #     # 右侧虚线（-45度）
            #     right_angle = arrow_angle - np.pi / 4
            #     right_u = np.cos(right_angle) * 5 * arrow_scale
            #     right_v = np.sin(right_angle) * 5 * arrow_scale
            #     ax.plot([x_arrows[i], x_arrows[i] + right_u], 
            #            [y_arrows[i], y_arrows[i] + right_v], 
            #            color=color, alpha=0.5, linestyle='--', linewidth=1)
    
    # 绘制3D box的ROI区域（在XY平面上的投影矩形）
    if box_center is not None and box_size is not None:
        box_center = np.asarray(box_center)
        box_size = np.asarray(box_size)
        box_size = box_size * 2
        
        # 计算矩形的左下角坐标和宽高（XY平面）
        rect_x = box_center[0] - box_size[0] / 2
        rect_y = box_center[1] - box_size[1] / 2
        rect_width = box_size[0]
        rect_height = box_size[1]
        
        # 绘制矩形框
        roi_rect = Rectangle((rect_x, rect_y), rect_width, rect_height,
                            linewidth=2, edgecolor='black', facecolor='none',
                            linestyle='--', label='ROI Box', zorder=10)
        ax.add_patch(roi_rect)
        
        # 标记box中心点
        ax.scatter(box_center[0], box_center[1], c='black', s=100, 
                  marker='x', linewidths=3, label='ROI Center', zorder=11)
    
    # 设置坐标轴
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    title = 'Multi-Camera Positions' if is_multi_camera else 'Camera Positions'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    if is_multi_camera or len(cameras_dict) == 1:
        ax.legend(loc='best', fontsize=10, markerscale=3)
    
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    
    # 自动调整坐标轴范围
    # if all_x and all_y:
    #     x_min, x_max = min(all_x), max(all_x)
    #     y_min, y_max = min(all_y), max(all_y)
    #     x_margin = (x_max - x_min) * 0.1
    #     y_margin = (y_max - y_min) * 0.1
    #     ax.set_xlim(x_min - x_margin, x_max + x_margin)
    #     ax.set_ylim(y_min - y_margin, y_max + y_margin)
    
    # 添加统计信息
    if is_multi_camera:
        info_text = f"Total cameras: {len(cameras_dict)}\n"
        for cam_name, cam_poses in cameras_dict.items():
            info_text += f"{cam_name}: {len(cam_poses)} poses\n"
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    else:
        plt.show()
    
    plt.close()

def clip_geojson(input_geojson_path, center_utm, radius_m, output_geojson_path):
    """
    根据 UTM 区域对输入 GeoJSON 处理
    - 点: 保留区域内的点
    - 线: 打断，保留区域范围内的部分
    - 面: 只要与区域相交，就保留整个 polygon
    """
    gdf = gpd.read_file(input_geojson_path)

    cx, cy = center_utm
    region = Point(cx, cy).buffer(radius_m)

    clipped_geoms = []
    for geom in gdf.geometry:
        if geom.is_empty:
            clipped_geoms.append(None)
            continue

        geom_type = geom.geom_type

        if geom_type in ["Point", "MultiPoint"]:
            if geom.within(region):
                clipped_geoms.append(geom)
            else:
                clipped_geoms.append(None)

        elif geom_type in ["LineString", "MultiLineString"]:
            inter = geom.intersection(region)
            if not inter.is_empty:
                clipped_geoms.append(inter)
            else:
                clipped_geoms.append(None)

        elif geom_type in ["Polygon", "MultiPolygon"]:
            if geom.intersects(region):
                clipped_geoms.append(geom)
            else:
                clipped_geoms.append(None)
        else:
            clipped_geoms.append(None)

    gdf["geometry"] = clipped_geoms
    gdf = gdf.dropna(subset=["geometry"])
    
    # 检查是否有有效数据
    if gdf.empty:
        print(f"⚠️  跳过 (区域内无数据): {os.path.basename(output_geojson_path)}")
        return False
    
    gdf.to_file(output_geojson_path, driver="GeoJSON")
    print(f"✅ 已输出 ({len(gdf)} 个要素): {os.path.basename(output_geojson_path)}")
    return True


def clip_hd_map_by_config(center_lon, center_lat, radius, input_folder, output_folder):
    print(f"✓ HD_MAP 输入文件夹: {input_folder}  ")
    print(f"✓ 输出文件夹: {output_folder}  ")
    if not output_folder:
        print("⚠ 错误: 未配置输出文件夹 hd_map.output_folder")
        exit(1)
    if center_lat is None or center_lon is None:
        raise ValueError("配置文件中缺少必需参数: search_params.default_location.latitude/longitude")
    if input_folder is None:
        raise ValueError("配置文件中缺少必需参数: hd_map.input_folder")
    if output_folder is None:
        raise ValueError("配置文件中缺少必需参数: hd_map.output_folder")
    print("="*80)
    print("高精地图裁切工具")
    print("="*80)
    print(f"输入文件夹: {input_folder}")
    print(f"输出文件夹: {output_folder}")
    print(f"中心点坐标: ({center_lat}, {center_lon})")
    print(f"裁切半径: {radius} 米")
    print("="*80)
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"✓ 已创建输出文件夹: {output_folder}")
    # 转换中心点坐标到UTM
    # x, y, epsg_code = wgs84_to_utm(center_lon, center_lat)
    x, y, epsg_code, hemisphere = lonlat_to_utm(center_lon, center_lat)
    center_utm = (x, y)
    print(f"✓ 自动检测到 UTM 带号 EPSG:{epsg_code}")
    print(f"✓ 中心点 UTM 坐标: ({x:.2f}, {y:.2f})")
    print("="*80)
    geojson_files = [f for f in os.listdir(input_folder) if f.endswith(".geojson")]
    geojson_files.sort()
    if not geojson_files:
        print(f"⚠ 警告: 在 {input_folder} 中没有找到 .geojson 文件")
        return
    print(f"找到 {len(geojson_files)} 个 GeoJSON 文件")
    print("="*80)
    success_count = 0
    skip_count = 0
    for idx, file in enumerate(geojson_files, 1):
        print(f"[{idx}/{len(geojson_files)}] 处理: {file}")
        file_path = os.path.join(input_folder, file)
        output_file_path = os.path.join(output_folder, file)
        print(f"裁切区域中心 UTM: ({center_utm[0]:.2f}, {center_utm[1]:.2f}), 半径: {radius} 米")
        print("输入文件:" + file_path)
        print("输出文件:" + output_file_path)

        try:
            if clip_geojson(file_path, center_utm, radius, output_file_path):
                success_count += 1
            else:
                skip_count += 1
        except Exception as e:
            print(f"❌ 处理失败: {file} - {e}")
            skip_count += 1
        print("文件输出到:")
        print(output_file_path)
    print("="*80)
    print(f"✓ 处理完成: {success_count} 个文件成功, {skip_count} 个文件跳过")
    print("="*80)