#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
 

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cout << "you should specify pcd file and voxel size . " << std::endl;
    return -1;
  }

  std::string filename = argv[1];
  std::cout << "filename: " << filename << std::endl;
  double voxel_size = std::stof(argv[2]);
  std::cout << "voxel_size: " << voxel_size << std::endl;

  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  PointCloudXYZI::Ptr cloud_out(new PointCloudXYZI);

  if (pcl::io::loadPCDFile(filename, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }

  pcl::VoxelGrid<PointType> downSizeFilterTempMap;
  downSizeFilterTempMap.setLeafSize(voxel_size, voxel_size, voxel_size);

  std::cout << "box points original:  " << cloud->size() << std::endl;
  downSizeFilterTempMap.setInputCloud(cloud);
  downSizeFilterTempMap.filter(*cloud_out);
  std::cout << "box points filter  :" << cloud_out->size() << std::endl;

  std::cout << "Saved voxel after: " << cloud_out->size() << " data points to downsample.pcd." << std::endl;
  pcl::io::savePCDFile("../downsample.pcd", *cloud_out);

  return (0);
}
