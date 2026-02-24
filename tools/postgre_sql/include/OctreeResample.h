#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>

// pcl
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils.h"

// triangle structure for mesh representation

// eigen
#include <Eigen/Eigen>

// OctreeResample 类：基于八叉树的非均匀体素滤波器
// 使用OctreePointCloudSearch保留原始点信息
template <typename PointT> class OctreeResample {
public:
  explicit OctreeResample(float voxel_size);

  void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

  void setRoadMeshTriangles(const std::vector<Triangle> &triangles);

  void buildOctree();

  void printVoxelCounts();

  void printOctreeStructure();

  typename pcl::PointCloud<PointT>::Ptr getFilteredCloud();

  // 获取内部八叉树对象的访问权限
  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr getOctree();

  // 非均匀采样方法
  typename pcl::PointCloud<PointT>::Ptr
  resample(const std::vector<Eigen::Vector3f> &roadPoints,
           std::vector<std::pair<float, float>> thresholds,
           bool isDynamicRemove = false);

private:
  float voxel_size_;
  typename pcl::PointCloud<PointT>::Ptr cloud_;
  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr octree_;

  //
  std::vector<Triangle> mesh_triangles_;

  std::vector<Triangle>
  extractCrossMesh(float blockMinX, float blockMinY, float blockSz,
                   const std::vector<Triangle> &mesh,
                   const std::vector<Eigen::Vector3f> &hdmapCloud);
};