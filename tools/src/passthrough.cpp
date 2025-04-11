#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/convex_hull.h>

// typedef pcl::PointXYZRGBA PointTypeRGB;
typedef pcl::PointXYZI PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "you should specify pcd file and voxel size . " << std::endl;
    return -1;
  }

  std::string filename = argv[1];
  std::cout << "filename: " << filename << std::endl;

  PointCloudXYZRGB::Ptr target_cloud(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);

  if (pcl::io::loadPCDFile(filename, *target_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }
  std::cout << "pts: " << target_cloud->size() << std::endl;

  std::cout << "start PassThrough: " << std::endl;
  // Create the filtering object
  pcl::PassThrough<PointTypeRGB> pass;
  pass.setInputCloud(target_cloud);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(-3.0, 0.5);
  pass.setNegative (true);
  // pass.setNegative(false); // 保留之内
  pass.filter(*cloud);
  std::cout << "filtered pts: " << cloud->size() << std::endl;

  pass.setInputCloud(cloud);
  pass.setFilterLimits(30, 200);
  pass.filter(*cloud);


  std::cout << "filtered pts: " << cloud->size() << std::endl;
  pcl::io::savePCDFile("/opt/csg/slam/navs/100.pcd", *cloud);
  return 0;

  // 创建离群点去除对象
  pcl::RadiusOutlierRemoval<PointTypeRGB> ror;
  ror.setInputCloud(target_cloud);
  ror.setRadiusSearch(0.10);      // 设置半径
  ror.setMinNeighborsInRadius(5); // 设置半径内的最小邻居数

  // 输出点云
  pcl::PointCloud<PointTypeRGB>::Ptr cloud_filtered(new pcl::PointCloud<PointTypeRGB>);
  ror.filter(*cloud);
  std::cout << "filtered pts: " << cloud->size() << std::endl;
  pcl::io::savePCDFile("/opt/csg/slam/navs/tt_11.pcd", *cloud);

  return (0);
}
