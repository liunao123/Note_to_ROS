"""
传感器参数管理模块

包含相机内参管理器和外参管理器，用于加载和管理多个传感器的标定参数。
"""

import numpy as np
import cv2
from pathlib import Path
from typing import Optional, List, Dict
from scipy.spatial.transform import Rotation


class SensorExtrinsicsManager:
    """传感器外参管理器，用于加载和管理多个传感器到vehicle坐标系的外参"""
    
    def __init__(self, extrinsics_dir=None):
        """
        初始化相机外参管理器
        
        Args:
            extrinsics_dir: 外参文件所在目录路径，如果为None则稍后通过load_from_directory加载
        """
        # 存储每个传感器到vehicle的变换矩阵 {sensor_name: T_vehicle_to_sensor}
        self._extrinsics_cache = {}
        # 存储原始数据（四元数和平移向量）
        self._raw_data_cache = {}
        
        if extrinsics_dir is not None:
            self.load_from_directory(extrinsics_dir)
    
    def load_from_directory(self, extrinsics_dir):
        """
        从目录加载所有外参文件
        
        Args:
            extrinsics_dir: 外参文件所在目录路径
        
        Returns:
            loaded_count: 成功加载的外参数量
        """
        extrinsics_dir = Path(extrinsics_dir)
        
        if not extrinsics_dir.exists():
            raise FileNotFoundError(f"外参目录不存在: {extrinsics_dir}")
        
        loaded_count = 0
        
        # 查找所有 .yaml 或 .yml 文件
        for yaml_file in extrinsics_dir.glob("*.yaml"):
            try:
                extrinsics = self._load_yaml_extrinsics(yaml_file)
                sensor_name = extrinsics['sensor_name']
                
                # 构建4x4变换矩阵
                T = self._build_transform_matrix(
                    extrinsics['quaternion'], 
                    extrinsics['translation']
                )
                
                self._extrinsics_cache[sensor_name] = T
                self._raw_data_cache[sensor_name] = extrinsics
                loaded_count += 1
                print(f"[OK] 加载外参: {sensor_name} -> vehicle <- {yaml_file.name}")
            except Exception as e:
                print(f"[WARN] 加载失败: {yaml_file.name}, 错误: {e}")
        
        for yml_file in extrinsics_dir.glob("*.yml"):
            try:
                extrinsics = self._load_yaml_extrinsics(yml_file)
                sensor_name = extrinsics['sensor_name']
                
                if sensor_name not in self._extrinsics_cache:
                    T = self._build_transform_matrix(
                        extrinsics['quaternion'], 
                        extrinsics['translation']
                    )
                    
                    self._extrinsics_cache[sensor_name] = T
                    self._raw_data_cache[sensor_name] = extrinsics
                    loaded_count += 1
                    print(f"[OK] 加载外参: {sensor_name} -> vehicle <- {yml_file.name}")
            except Exception as e:
                print(f"[WARN] 加载失败: {yml_file.name}, 错误: {e}")
        
        print(f"[INFO] 共加载 {loaded_count} 个外参")
        return loaded_count
    
    def _load_yaml_extrinsics(self, yaml_path):
        """
        从YAML文件加载外参
        
        Args:
            yaml_path: YAML文件路径
        
        Returns:
            extrinsics: dict包含外参信息
        """
        fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
        
        if not fs.isOpened():
            raise ValueError(f"无法打开文件: {yaml_path}")
        
        try:
            # 读取传感器信息
            sensor_name_node = fs.getNode('sensor_name')
            sensor_name = sensor_name_node.string() if sensor_name_node.isNone() == False else None
            
            sensor_type_node = fs.getNode('sensor_type')
            sensor_type = sensor_type_node.string() if sensor_type_node.isNone() == False else None
            
            sensor_id_node = fs.getNode('sensor_id')
            sensor_id = int(sensor_id_node.real()) if sensor_id_node.isNone() == False else None
            
            # 读取四元数 (w, x, y, z)
            quat = fs.getNode('r_quaternion_wxyz').mat()
            if quat is None:
                raise ValueError(f"未找到r_quaternion_wxyz字段: {yaml_path}")
            
            # 读取平移向量 (x, y, z)
            trans = fs.getNode('t_metric_xyz').mat()
            if trans is None:
                raise ValueError(f"未找到t_metric_xyz字段: {yaml_path}")
            
            # 读取其他信息
            calib_time_node = fs.getNode('calibration_time')
            calib_time = calib_time_node.string() if calib_time_node.isNone() == False else None
            
            coord_sys_node = fs.getNode('vehicle_coordinate_system')
            coord_sys = coord_sys_node.string() if coord_sys_node.isNone() == False else None
            
        finally:
            fs.release()
        
        # 转换为numpy数组
        quaternion = quat.flatten()  # [w, x, y, z]
        translation = trans.flatten()  # [x, y, z]
        
        extrinsics = {
            'sensor_name': sensor_name,
            'sensor_type': sensor_type,
            'sensor_id': sensor_id,
            'quaternion': quaternion,
            'translation': translation,
            'calibration_time': calib_time,
            'coordinate_system': coord_sys
        }
        
        return extrinsics
    
    def _build_transform_matrix(self, quaternion, translation):
        """
        从四元数和平移向量构建4x4变换矩阵
        
        Args:
            quaternion: (w, x, y, z) 四元数
            translation: (x, y, z) 平移向量
        
        Returns:
            T: 4x4变换矩阵
        """
        w, x, y, z = quaternion
        
        # 四元数转旋转矩阵
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        
        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation
        
        return T
    
    def get_transform(self, from_frame, to_frame='vehicle'):
        """
        获取从from_frame到to_frame的变换矩阵
        
        使用变换矩阵链式法则：
        - 如果都是传感器坐标系，则通过vehicle作为中转
        - T_to_from = T_to_vehicle @ T_vehicle_from
        
        Args:
            from_frame: 源坐标系名称（传感器名称或'vehicle'）
            to_frame: 目标坐标系名称（传感器名称或'vehicle'），默认'vehicle'
        
        Returns:
            T: 4x4变换矩阵，从from_frame到to_frame的变换
            如果找不到对应的坐标系，返回None
        """
        # 处理特殊情况：同一坐标系
        if from_frame == to_frame:
            return np.eye(4)
        
        # 情况1: from_frame -> vehicle
        if to_frame == 'vehicle':
            if from_frame not in self._extrinsics_cache:
                print(f"[WARN] 未找到坐标系: {from_frame}")
                return None
            # 文件中存储的是sensor到vehicle的变换
            return self._extrinsics_cache[from_frame].copy()
        
        # 情况2: vehicle -> to_frame
        if from_frame == 'vehicle':
            if to_frame not in self._extrinsics_cache:
                print(f"[WARN] 未找到坐标系: {to_frame}")
                return None
            # 返回逆变换
            T_to_vehicle = self._extrinsics_cache[to_frame]
            return np.linalg.inv(T_to_vehicle)
        
        # 情况3: sensor1 -> sensor2 (通过vehicle中转)
        if from_frame not in self._extrinsics_cache:
            print(f"[WARN] 未找到坐标系: {from_frame}")
            return None
        if to_frame not in self._extrinsics_cache:
            print(f"[WARN] 未找到坐标系: {to_frame}")
            return None
        
        # T_to_from = T_to_vehicle @ inv(T_from_vehicle)
        T_from_vehicle = self._extrinsics_cache[from_frame]
        T_to_vehicle = self._extrinsics_cache[to_frame]
        
        # sensor1 -> vehicle -> sensor2
        T_to_from = np.linalg.inv(T_to_vehicle) @ T_from_vehicle
        
        return T_to_from
    
    def get_pose(self, from_frame, to_frame='vehicle'):
        """
        获取从from_frame到to_frame的pose（平移和旋转）
        
        Args:
            from_frame: 源坐标系名称
            to_frame: 目标坐标系名称，默认'vehicle'
        
        Returns:
            pose: dict包含:
                - 'translation': (x, y, z) 平移向量
                - 'rotation_matrix': 3x3旋转矩阵
                - 'quaternion': (w, x, y, z) 四元数
            如果找不到对应的坐标系，返回None
        """
        T = self.get_transform(from_frame, to_frame)
        
        if T is None:
            return None
        
        translation = T[:3, 3]
        rotation_matrix = T[:3, :3]
        
        # 旋转矩阵转四元数（使用scipy的方法）
        r = Rotation.from_matrix(rotation_matrix)
        quaternion = r.as_quat()  # 返回 [x, y, z, w]
        quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])  # 转为 [w, x, y, z]
        
        pose = {
            'translation': translation,
            'rotation_matrix': rotation_matrix,
            'quaternion': quaternion
        }
        
        return pose
    
    def get_sensor_names(self):
        """获取所有已加载的传感器名称列表"""
        return list(self._extrinsics_cache.keys())
    
    def has_sensor(self, sensor_name):
        """检查是否存在指定传感器的外参"""
        return sensor_name in self._extrinsics_cache
    
    def get_raw_data(self, sensor_name):
        """获取传感器的原始外参数据"""
        return self._raw_data_cache.get(sensor_name)
    
    def __len__(self):
        """返回已加载的外参数量"""
        return len(self._extrinsics_cache)
    
    def __repr__(self):
        sensors = ', '.join(self._extrinsics_cache.keys())
        return f"SensorExtrinsicsManager({len(self)} sensors: {sensors})"


class CameraIntrinsicsManager:
    """相机内参管理器，用于加载和管理多个相机的内参"""
    
    def __init__(self, intrinsics_dir=None):
        """
        初始化相机内参管理器
        
        Args:
            intrinsics_dir: 内参文件所在目录路径，如果为None则稍后通过load_from_directory加载
        """
        self._intrinsics_cache = {}
        
        if intrinsics_dir is not None:
            self.load_from_directory(intrinsics_dir)
    
    def load_from_directory(self, intrinsics_dir):
        """
        从目录加载所有相机内参文件
        
        Args:
            intrinsics_dir: 内参文件所在目录路径
        
        Returns:
            loaded_count: 成功加载的内参数量
        """
        intrinsics_dir = Path(intrinsics_dir)
        
        if not intrinsics_dir.exists():
            raise FileNotFoundError(f"内参目录不存在: {intrinsics_dir}")
        
        loaded_count = 0
        
        # 查找所有 .yaml 或 .yml 文件
        for yaml_file in intrinsics_dir.glob("*.yaml"):
            try:
                camera_name = self._parse_camera_name(yaml_file.name)
                intrinsics = self._load_yaml_intrinsics(yaml_file)
                self._intrinsics_cache[camera_name] = intrinsics
                loaded_count += 1
                print(f"[OK] 加载相机内参: {camera_name} <- {yaml_file.name}")
            except Exception as e:
                print(f"[WARN] 加载失败: {yaml_file.name}, 错误: {e}")
        
        for yml_file in intrinsics_dir.glob("*.yml"):
            try:
                camera_name = self._parse_camera_name(yml_file.name)
                if camera_name not in self._intrinsics_cache:
                    intrinsics = self._load_yaml_intrinsics(yml_file)
                    self._intrinsics_cache[camera_name] = intrinsics
                    loaded_count += 1
                    print(f"[OK] 加载相机内参: {camera_name} <- {yml_file.name}")
            except Exception as e:
                print(f"[WARN] 加载失败: {yml_file.name}, 错误: {e}")
        
        print(f"[INFO] 共加载 {loaded_count} 个相机内参")
        return loaded_count
    
    def _parse_camera_name(self, filename):
        """
        从文件名解析相机名称
        
        例如: cam1_Intrinsic.yaml -> cam1
        """
        # 去除扩展名
        name = filename.replace('.yaml', '').replace('.yml', '')
        
        # 去除常见后缀
        name = name.replace('_Intrinsic', '').replace('_intrinsic', '')
        name = name.replace('_Intrinsics', '').replace('_intrinsics', '')
        
        return name
    
    def _load_yaml_intrinsics(self, yaml_path):
        """
        从YAML文件加载相机内参
        
        Args:
            yaml_path: YAML文件路径
        
        Returns:
            intrinsics: dict包含内参信息
        """
        # 使用OpenCV的FileStorage读取，因为是OpenCV格式的YAML
        fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
        
        if not fs.isOpened():
            raise ValueError(f"无法打开文件: {yaml_path}")
        
        try:
            # 读取相机矩阵
            K = fs.getNode('cameraMatrix').mat()
            if K is None:
                raise ValueError(f"未找到cameraMatrix字段: {yaml_path}")
            
            # 读取畸变系数
            dist_coeffs_node = fs.getNode('distCoeffs')
            dist_coeffs = dist_coeffs_node.mat() if dist_coeffs_node.isNone() == False else None
            
            # 读取图像尺寸
            image_width_node = fs.getNode('ImageWidth')
            image_height_node = fs.getNode('ImageHeight')
            
            image_width = int(image_width_node.real()) if image_width_node.isNone() == False else None
            image_height = int(image_height_node.real()) if image_height_node.isNone() == False else None
            
            # 读取其他信息
            model_node = fs.getNode('Model')
            type_node = fs.getNode('Type')
            
            model = model_node.string() if model_node.isNone() == False else None
            cam_type = type_node.string() if type_node.isNone() == False else None
            
        finally:
            fs.release()
        
        # 提取fx, fy, cx, cy
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        
        intrinsics = {
            'K': K,
            'fx': fx,
            'fy': fy,
            'cx': cx,
            'cy': cy,
            'camera_matrix': K,
            'dist_coeffs': dist_coeffs,
            'image_width': image_width,
            'image_height': image_height,
            'model': model,
            'type': cam_type
        }
        
        return intrinsics
    
    def get_intrinsics(self, camera_name):
        """
        获取指定相机的内参
        
        Args:
            camera_name: 相机名称，例如 "cam1"
        
        Returns:
            intrinsics: dict包含内参信息，如果未找到则返回None
        """
        return self._intrinsics_cache.get(camera_name)
    
    def get_all_camera_names(self):
        """获取所有已加载的相机名称列表"""
        return list(self._intrinsics_cache.keys())
    
    def has_camera(self, camera_name):
        """检查是否存在指定相机的内参"""
        return camera_name in self._intrinsics_cache
    
    def __len__(self):
        """返回已加载的相机数量"""
        return len(self._intrinsics_cache)
    
    def __repr__(self):
        cameras = ', '.join(self._intrinsics_cache.keys())
        return f"CameraIntrinsicsManager({len(self)} cameras: {cameras})"
