'''
    生成天空点云的初始化，该点云在一个球形的界面上，模拟天空的形状，并且，将该点云放在较远的地方，
'''

import numpy as np

class SkyPointCloud:
    def __init__(self, num_points=1000, radius=100.0, color=(135, 206, 235), center=(0.0, 0.0, 0.0)):
        """
        初始化天空点云
        :param num_points: 点的数量
        :param radius: 球面半径，决定点云距离
        :param color: 点的颜色, RGB元组
        :param center: 场景中心点坐标 (x, y, z)
        """
        self.num_points = num_points
        self.radius = radius
        self.color = color
        self.center = np.array(center)
        self.points = self._generate_points()
        self.colors = np.tile(np.array(color), (num_points, 1))

    def _generate_points(self):
        """
        在以center为中心的球面上均匀采样点，只保留z > center[2]的点
        """
        points = []
        while len(points) < self.num_points:
            phi = np.random.uniform(0, 2 * np.pi)
            costheta = np.random.uniform(-1, 1)
            theta = np.arccos(costheta)
            x = self.radius * np.sin(theta) * np.cos(phi)
            y = self.radius * np.sin(theta) * np.sin(phi)
            z = self.radius * np.cos(theta)
            point = np.array([x, y, z]) + self.center
            if point[2] > self.center[2]:  # 只保留上半球
                points.append(point)
        return np.array(points)

    def get_point_cloud(self):
        """
        返回点云的坐标和颜色
        """
        return self.points, self.colors

    def save_as_ply(self, filename):
        """
        保存点云为PLY文件，包含法向量
        """
        with open(filename, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {self.num_points}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property uchar red\n')
            f.write('property uchar green\n')
            f.write('property uchar blue\n')
            f.write('property float nx\n')
            f.write('property float ny\n')
            f.write('property float nz\n')
            f.write('end_header\n')
            for i in range(self.num_points):
                x, y, z = self.points[i]
                r, g, b = self.colors[i]
                # 法向量：点到球心的方向归一化
                normal = np.array([x, y, z]) - self.center
                normal = normal / np.linalg.norm(normal)
                nx, ny, nz = normal
                f.write(f'{x} {y} {z} {int(r)} {int(g)} {int(b)} {nx} {ny} {nz}\n')

# # 示例用法
# if __name__ == "__main__":
#     # 场景中心点假设为 (10, 20, 5)
#     sky = SkyPointCloud(num_points=500000, radius=10000, color=(135, 206, 250), center=(0, 0, 0))
#     sky.save_as_ply("/data/3dgs_data_grid/sky.ply")


