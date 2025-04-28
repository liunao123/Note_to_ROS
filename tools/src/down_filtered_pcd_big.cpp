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
#include <pcl/filters/crop_box.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cout << "you should specify pcd file and voxel size . "  << std::endl;
    return -1;
  }

  std::string filename = argv[1];
  std::cout << "filename: " << filename << std::endl;
  double voxel_size = std::stof(argv[2]);
  std::cout << "voxel_size: " << voxel_size << std::endl;

    // 创建网格
    const double grid_size = voxel_size * 1000.0;  // 10.0; // 10米
  std::cout << "grid_size: " << grid_size << std::endl;


  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  PointCloudXYZI::Ptr cloud_out(new PointCloudXYZI);

  if (pcl::io::loadPCDFile(filename, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }
  std::cout << "pts: " << cloud->size() << std::endl;

    PointType min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    double min_x = min_pt.x;
    double min_y = min_pt.y;
    double min_z = min_pt.z;
    double max_x = max_pt.x;
    double max_y = max_pt.y;
    double max_z = max_pt.z;

    pcl::CropBox< PointType > cropBoxFilter_temp(false);

  pcl::VoxelGrid<PointType> downSizeFilterTempMap;
  downSizeFilterTempMap.setLeafSize(voxel_size, voxel_size, voxel_size);
  
    PointCloudXYZI::Ptr cloud_crop(new PointCloudXYZI);

    for (double x = min_x; x < max_x; x += grid_size) {
        for (double y = min_y; y < max_y; y += grid_size) {
            for (double z = min_z; z < max_z; z += grid_size) {

  cropBoxFilter_temp.setMin(Eigen::Vector4f(x , y, z, 1.0f));
  cropBoxFilter_temp.setMax(Eigen::Vector4f(x + grid_size, y + grid_size, z + grid_size, 1.0f));
  cropBoxFilter_temp.setInputCloud(cloud);
  cropBoxFilter_temp.filter(*cloud_crop);

  if( cloud_crop->size() < 10 )
    continue;

    std::cout << "points.clear: " << cloud_crop->size() << std::endl;
  downSizeFilterTempMap.setInputCloud(cloud_crop);
  downSizeFilterTempMap.filter(*cloud_crop);
    std::cout << "points.clear: " << cloud_crop->size() << std::endl;
    *cloud_out += *cloud_crop;
    std::cout << "cloud_out.clear: " << cloud_out->size() << std::endl;
 
            }
        }
    }

  std::cout << "Saved voxel after: " << cloud_out->size() << " data points to test_pcd.pcd." << std::endl;
  pcl::io::savePCDFile("/opt/csg/slam/navs/downsample.pcd", *cloud_out);

  return (0);
}
