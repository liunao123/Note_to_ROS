#include "OctreeResample.h"

template <typename PointT>
OctreeResample<PointT>::OctreeResample(float voxel_size)
    : voxel_size_(voxel_size), cloud_(nullptr) {}

template <typename PointT>
void OctreeResample<PointT>::setInputCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  cloud_ = cloud;
}

template <typename PointT> void OctreeResample<PointT>::buildOctree() {
  if (!cloud_) {
    std::cerr << "Input cloud is empty!" << std::endl;
    return;
  }

  // 使用 PCL 的八叉树结构
  octree_.reset(new pcl::octree::OctreePointCloudSearch<PointT>(voxel_size_));
  octree_->setInputCloud(cloud_);
  octree_->addPointsFromInputCloud();

  std::cout << "Octree built with voxel size: " << voxel_size_ << std::endl;
}

template <typename PointT> void OctreeResample<PointT>::printVoxelCounts() {
  if (!octree_) {
    std::cerr << "Octree not built yet!" << std::endl;
    return;
  }

  size_t leaf_count = 0;
  for (auto it = octree_->leaf_depth_begin(); it != octree_->leaf_depth_end();
       ++it) {
    leaf_count++;
  }

  std::cout << "Total leaf nodes (voxels): " << leaf_count << std::endl;
  std::cout << "Original point count: " << cloud_->size() << std::endl;
}

template <typename PointT> void OctreeResample<PointT>::printOctreeStructure() {
  if (!octree_) {
    std::cerr << "Octree not built yet!" << std::endl;
    return;
  }

  // 获取八叉树的基本信息
  unsigned int max_depth = octree_->getTreeDepth();
  size_t branch_count = octree_->getBranchCount();
  size_t leaf_count = octree_->getLeafCount();

  std::cout << "八叉树基本信息:" << std::endl;
  std::cout << "  - 最大深度: " << max_depth << " 层" << std::endl;
  std::cout << "  - 分支节点数: " << branch_count << std::endl;
  std::cout << "  - 叶子节点数: " << leaf_count << std::endl;
  std::cout << "  - 总节点数: " << (branch_count + leaf_count) << std::endl;
  std::cout << "  - 体素分辨率: " << voxel_size_ << " m" << std::endl;
  std::cout << "  - 输入点云数量: " << cloud_->size() << std::endl;

  // 统计每层的所有节点数量（包括分支节点和叶子节点）
  std::map<unsigned int, size_t> depth_all_node_count;
  std::map<unsigned int, size_t> depth_leaf_node_count;
  std::map<unsigned int, size_t> depth_point_count;

  // 统计所有节点（包括分支节点）
  for (auto it = octree_->breadth_begin(); it != octree_->breadth_end(); ++it) {
    unsigned int depth = it.getCurrentOctreeDepth();
    depth_all_node_count[depth]++;
  }

  // 统计叶子节点和点数
  for (auto it = octree_->leaf_depth_begin(); it != octree_->leaf_depth_end();
       ++it) {
    unsigned int depth = it.getCurrentOctreeDepth();
    pcl::IndicesPtr indexVector(new std::vector<int>);
    pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();
    container.getPointIndices(*indexVector);

    depth_leaf_node_count[depth]++;
    depth_point_count[depth] += indexVector->size();
  }

  std::cout << "\n各层节点统计:" << std::endl;
  std::cout << "  层级 | 总节点数 | 叶子节点 | 分支节点 | 点数 | "
               "平均点数/叶子 | 该层体素大小(m)"
            << std::endl;
  std::cout << "  " << std::string(85, '-') << std::endl;

  size_t total_nodes_sum = 0;
  size_t total_leaves_sum = 0;
  size_t total_points_sum = 0;

  for (unsigned int depth = 0; depth <= max_depth; depth++) {
    size_t all_nodes = depth_all_node_count[depth];
    size_t leaf_nodes = depth_leaf_node_count[depth];
    size_t branch_nodes = all_nodes - leaf_nodes;
    size_t points = depth_point_count[depth];
    double avg_points =
        leaf_nodes > 0 ? static_cast<double>(points) / leaf_nodes : 0.0;
    double layer_voxel_size = voxel_size_ * std::pow(2.0, max_depth - depth);

    total_nodes_sum += all_nodes;
    total_leaves_sum += leaf_nodes;
    total_points_sum += points;

    std::cout << "  " << std::setw(4) << depth << " | " << std::setw(8)
              << all_nodes << " | " << std::setw(8) << leaf_nodes << " | "
              << std::setw(8) << branch_nodes << " | " << std::setw(8) << points
              << " | " << std::setw(14) << std::fixed << std::setprecision(2)
              << avg_points << " | " << std::setw(10) << std::setprecision(4)
              << layer_voxel_size << std::endl;
  }

  std::cout << "  " << std::string(85, '-') << std::endl;
  std::cout << "  合计 | " << std::setw(8) << total_nodes_sum << " | "
            << std::setw(8) << total_leaves_sum << " | " << std::setw(8)
            << (total_nodes_sum - total_leaves_sum) << " | " << std::setw(8)
            << total_points_sum << " |" << std::endl;

  // 获取边界框信息
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);

  std::cout << "\n空间边界:" << std::endl;
  std::cout << "  X: [" << std::fixed << std::setprecision(3) << min_x << ", "
            << max_x << "] 范围: " << (max_x - min_x) << " m" << std::endl;
  std::cout << "  Y: [" << min_y << ", " << max_y
            << "] 范围: " << (max_y - min_y) << " m" << std::endl;
  std::cout << "  Z: [" << min_z << ", " << max_z
            << "] 范围: " << (max_z - min_z) << " m" << std::endl;

  // 计算压缩率
  double compression_ratio =
      static_cast<double>(leaf_count) / cloud_->size() * 100.0;
  std::cout << "\n压缩统计:" << std::endl;
  std::cout << "  压缩率: " << std::setprecision(2) << compression_ratio
            << "% (保留 " << leaf_count << " / " << cloud_->size() << " 点)"
            << std::endl;
}

template <typename PointT>
void OctreeResample<PointT>::setRoadMeshTriangles(
    const std::vector<Triangle> &triangles) {
  mesh_triangles_ = triangles;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
OctreeResample<PointT>::getFilteredCloud() {
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud(
      new pcl::PointCloud<PointT>());

  if (!octree_) {
    std::cerr << "Octree not built yet!" << std::endl;
    return filtered_cloud;
  }

  // 从每个体素中提取代表点
  for (auto it = octree_->leaf_depth_begin(); it != octree_->leaf_depth_end();
       ++it) {
    pcl::IndicesPtr indexVector(new std::vector<int>);
    pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();
    container.getPointIndices(*indexVector);

    if (!indexVector->empty()) {
      // 取体素中的第一个点作为代表
      // filtered_cloud->push_back(cloud_->points[(*indexVector)[0]]);

      // 取体素的几何中心作为代表
      PointT centroid;
      centroid.x = 0.0f;
      centroid.y = 0.0f;
      centroid.z = 0.0f;

      if constexpr (pcl::traits::has_color<PointT>::value) {
        centroid.r = 0;
        centroid.g = 0;
        centroid.b = 0;
      }

      if constexpr (pcl::traits::has_intensity<PointT>::value) {
        centroid.intensity = 0.0f;
      }

      uint32_t r_sum = 0, g_sum = 0, b_sum = 0;
      float intensity_sum = 0.0f;

      // 累加所有点的坐标
      for (const auto &idx : *indexVector) {
        const PointT &pt = cloud_->points[idx];
        centroid.x += pt.x;
        centroid.y += pt.y;
        centroid.z += pt.z;

        if constexpr (pcl::traits::has_color<PointT>::value) {
          r_sum += pt.r;
          g_sum += pt.g;
          b_sum += pt.b;
        }

        if constexpr (pcl::traits::has_intensity<PointT>::value) {
          intensity_sum += pt.intensity;
        }
      }

      // 计算平均值得到几何中心
      size_t count = indexVector->size();
      centroid.x /= count;
      centroid.y /= count;
      centroid.z /= count;

      if constexpr (pcl::traits::has_color<PointT>::value) {
        centroid.r = static_cast<uint8_t>(r_sum / count);
        centroid.g = static_cast<uint8_t>(g_sum / count);
        centroid.b = static_cast<uint8_t>(b_sum / count);
      }

      if constexpr (pcl::traits::has_intensity<PointT>::value) {
        centroid.intensity = intensity_sum / count;
      }

      filtered_cloud->push_back(centroid);
    }
  }

  filtered_cloud->width = filtered_cloud->size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;

  return filtered_cloud;
}

template <typename PointT>
typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr
OctreeResample<PointT>::getOctree() {
  return octree_;
}

// 显式实例化
template class OctreeResample<pcl::PointXYZ>;
template class OctreeResample<pcl::PointXYZI>;
template class OctreeResample<pcl::PointXYZRGB>;
template class OctreeResample<pcl::PointXYZRGBA>;

template <typename PointT>
std::vector<Triangle> OctreeResample<PointT>::extractCrossMesh(
    float blockMinX, float blockMinY, float blockSz,
    const std::vector<Triangle> &mesh,
    const std::vector<Eigen::Vector3f> &hdmapCloud) {

  //从hdmapCloud中提取当前block范围内的点
  std::vector<Eigen::Vector3f> hdmapBlockPoints;
  for (const auto &pt : hdmapCloud) {
    if (pt.x() >= blockMinX && pt.x() <= blockMinX + blockSz &&
        pt.y() >= blockMinY && pt.y() <= blockMinY + blockSz) {
      hdmapBlockPoints.push_back(pt);
    }
  }

  // 从所有mesh triangles中提取与kps相交的三角形
  std::vector<Triangle> rtn;
  for (const auto &triangle : mesh) {
    Eigen::Vector3f p0, p1, p2;
    p0 = hdmapCloud[triangle.v0];
    p1 = hdmapCloud[triangle.v1];
    p2 = hdmapCloud[triangle.v2];
    for (auto &kp : hdmapBlockPoints) {
      if (kp == p0 || kp == p1 || kp == p2) {
        rtn.emplace_back(triangle);
        break;
      }
    }
  }
  return rtn;
}

// 非均匀采样实现
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr OctreeResample<PointT>::resample(
    const std::vector<Eigen::Vector3f> &roadPoints,
    std::vector<std::pair<float, float>> thresholds, bool isDynamicRemove) {

  if (!octree_) {
    return nullptr;
  }

  if (thresholds.size() < 1) {
    return nullptr;
  }

  typename pcl::PointCloud<PointT>::Ptr resultCloud(
      new pcl::PointCloud<PointT>);

  // thresholds : first is min distance, second is voxel size
  //  sort thresholds by first value
  std::sort(thresholds.begin(), thresholds.end(),
            [](const std::pair<float, float> &a,
               const std::pair<float, float> &b) { return a.first < b.first; });

  int totalPoints = 0;
  std::map<float, int> leafNums;
  for (const auto &th : thresholds) {
    leafNums[th.first] = 0;
  }

  // 计算体素中心位置
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);

  //遍历所有叶子节点
  for (auto it = octree_->breadth_begin(); it != octree_->breadth_end(); ++it) {
    if (!it.isLeafNode()) {
      continue;
    }

    unsigned int curDepth = it.getCurrentOctreeDepth();
    const pcl::octree::OctreeKey &key = it.getCurrentOctreeKey();
    double voxelSz = std::sqrt(octree_->getVoxelSquaredSideLen(curDepth));

    float centerX = min_x + (key.x + 0.5) * voxelSz;
    float centerY = min_y + (key.y + 0.5) * voxelSz;
    float centerZ = min_z + (key.z + 0.5) * voxelSz;

    Eigen::Vector3f voxelCenter;
    voxelCenter << centerX, centerY, centerZ;

    float minDist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < roadPoints.size(); i++) {
      // float d = (voxelCenter - roadPoints[i]).norm();
      float d = std::sqrt((voxelCenter.x() - roadPoints[i].x()) * (voxelCenter.x() - roadPoints[i].x()) +
                          (voxelCenter.y() - roadPoints[i].y()) * (voxelCenter.y() - roadPoints[i].y()) );
      if (minDist > d) {
        minDist = d;
      }
    }

    // 检查minDist落在哪个阈值区间内
    float voxelSize = -1;
    for (const auto &threshold : thresholds) {
      if (minDist <= threshold.first) {
        voxelSize = threshold.second;
        leafNums[threshold.first]++;
        break;
      }
    }

    if (voxelSize < 0) {
      continue;
    }

    // run voxel grid filter
    pcl::IndicesPtr indexVector(new std::vector<int>);
    pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();
    container.getPointIndices(*indexVector);

    totalPoints += indexVector->size();

    if (indexVector->empty()) {
      continue;
    }

    typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_);
    vg.setIndices(indexVector);
    vg.setLeafSize(voxelSize, voxelSize, voxelSize);
    vg.filter(*filtered);

    // if (0)
    // todo 只对地面附近的点云进行离群点移除  平面距离即可
    if (voxelSize < 0.15)
    {
      // 靠近行车道的点云，应该点数较多，// 如果点数过少，说明是空中漂浮的点，直接丢弃 
      // ! 效果很好 20260319 by ln
      if (filtered->size() < 15)
      {
         filtered->points.clear();
         filtered->width = 0;
         filtered->height = 1;
         filtered->is_dense = true;
         std::cout << "Node | Points too less . clear all pts  ...... " << std::endl;
         continue;
      }
      std::cout << "Node | Points Before: " << filtered->size() << std::endl;
      const int mean_k = 20;
      const double stddev_mul_thresh = 2.0;
      pcl::StatisticalOutlierRemoval< PointT > sor;
      sor.setInputCloud(filtered);
      sor.setMeanK(mean_k);
      sor.setStddevMulThresh(stddev_mul_thresh);
      sor.filter(*filtered);
      std::cout << "Node | Points After SOR: " << filtered->size() << std::endl;
    }

    // 判断是否在行车道范围内，对车道范围内的点云做特殊处理
    if (minDist <= voxel_size_ && isDynamicRemove) {
      // 1. 提取相交mesh
      std::vector<Triangle> crossedTriangles =
          extractCrossMesh(centerX - voxelSz / 2, centerY - voxelSz / 2,
                           voxelSz, mesh_triangles_, roadPoints);

      // 2. 估算plane functions（重复运算）
      std::vector<Eigen::Vector4f> planes;
      for (size_t j = 0; j < crossedTriangles.size(); j++) {
        Eigen::Vector3f p0, p1, p2;
        p0 = roadPoints[crossedTriangles[j].v0];
        p1 = roadPoints[crossedTriangles[j].v1];
        p2 = roadPoints[crossedTriangles[j].v2];
        Eigen::Vector4f func;
        if (FitPlaneFromTriangle(p0, p1, p2, func)) {
          planes.emplace_back(func);
        } else {
          planes.emplace_back(Eigen::Vector4f::Zero());
        }
      }

      // 3. 点分类：1. 行车道路面点， 2. 行车道上空点， 3. 非行车道点
      int removedPoints = 0;
      for (auto &pt : *filtered) {
        bool goodPoint = true;
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        for (size_t j = 0; j < crossedTriangles.size(); j++) {
          Eigen::Vector3f p0, p1, p2, projFoot;
          p0 = roadPoints[crossedTriangles[j].v0];
          p1 = roadPoints[crossedTriangles[j].v1];
          p2 = roadPoints[crossedTriangles[j].v2];

          int label = pointClassify(p, p0, p1, p2, planes[j], projFoot, 0.2f);
          if (label == 3) {
            goodPoint = true;
            break;
          } else if (label == 1 || label == 0) {
            goodPoint = false;
            break;
          } else {
            continue;
          }
        }
        if (!goodPoint) {
          pt.x = std::numeric_limits<float>::quiet_NaN();
          pt.y = std::numeric_limits<float>::quiet_NaN();
          pt.z = std::numeric_limits<float>::quiet_NaN();
          removedPoints++;
        }
      }

      // 移除NaN点
      filtered->is_dense = false;
      typename pcl::PointCloud<PointT>::Ptr nanFiltered(
          new pcl::PointCloud<PointT>);
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*filtered, *nanFiltered, indices);
      *resultCloud += *nanFiltered;

      std::cout << "removed points in road mesh filtering: " << filtered->size()
                << " -> " << nanFiltered->size() << " (marked " << removedPoints
                << " as NaN)" << std::endl;
    } else {
      *resultCloud += *filtered;
    }
  }

  std::cout << "Total points before filtering: " << totalPoints << std::endl;
  for (const auto &ln : leafNums) {
    std::cout << "Threshold distance: " << ln.first
              << ", Leaf nodes processed: " << ln.second << std::endl;
  }

  return resultCloud;
}