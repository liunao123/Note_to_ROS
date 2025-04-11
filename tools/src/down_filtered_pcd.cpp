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
// #include <pcl/surface/alpha_shape.h>

// typedef pcl::PointXYZRGBA PointTypeRGB;
typedef pcl::PointXYZI PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

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

  PointCloudXYZRGB::Ptr target_cloud(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_in(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);

  if (pcl::io::loadPCDFile(filename, *target_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }
  std::cout << "pts: " << target_cloud->size() << std::endl;
  std::cout << "voxel_size: " << voxel_size << std::endl;

  // pcl::CropBox< PointTypeRGB > cropBoxFilter_temp;
  // float range = 120.0;
  // cropBoxFilter_temp.setMin(Eigen::Vector4f(-25.0f, -40.0f, -3.0f, 1.0f));
  // cropBoxFilter_temp.setMax(Eigen::Vector4f(25.0f, 40.0f, 12.0f, 1.0f));
  // cropBoxFilter_temp.setInputCloud(target_cloud);

  // // 之外的点很稀疏，不参与滤波
  // // cropBoxFilter_temp.setNegative(true); // 保留 range 之 外 的 点
  // // cropBoxFilter_temp.filter(*cloud_out);
  // // pcl::io::savePCDFile("/home/cloud_out.pcd", *cloud_out);
  
  // cropBoxFilter_temp.setNegative(false); // 保留 range 之内的 点
  // cropBoxFilter_temp.filter(*target_cloud);
  // // pcl::io::savePCDFile("/home/cloud_in.pcd", *target_cloud);
  // std::cout << "pts: " << target_cloud->size() << std::endl;

  pcl::VoxelGrid<PointTypeRGB> downSizeFilterTempMap;
  downSizeFilterTempMap.setLeafSize(voxel_size, voxel_size, voxel_size);
  downSizeFilterTempMap.setInputCloud(target_cloud);
  downSizeFilterTempMap.filter(*cloud);
  std::cout << "Saved voxel after: " << cloud->size() << " data points to test_pcd.pcd." << std::endl;
  pcl::io::savePCDFile("/opt/csg/slam/navs/downsample.pcd", *cloud);

  return 0;

  // std::cout << "pts: " << cloud->size()  << std::endl;
  // std::cout << "start savePCDFile: "  << std::endl;
  // pcl::io::savePCDFile("/home/map/hf_intensity_gt80.pcd", *cloud);
  return (1);

  PointCloudXYZRGB output_cloud;
  for (size_t i = 0; i < 1; i++)
  {
    // voxel_size = double(i) / 100.0 ;
    std::cout << "voxel_size: " << voxel_size << std::endl;

    std::cout << "Saved voxel before: " << cloud_in->size() << " data points to test_pcd.pcd." << std::endl;
    downSizeFilterTempMap.setLeafSize( voxel_size , voxel_size , voxel_size  );
    downSizeFilterTempMap.setInputCloud(cloud_in);
    downSizeFilterTempMap.filter(*cloud);

    std::cout << "Saved voxel after: " << cloud->size() << " data points to test_pcd.pcd." << std::endl;
    *cloud += *cloud_out;
    std::cout << "Saved voxel after: " << cloud->size() << " data points to test_pcd.pcd." << std::endl;

    // pcl::io::savePCDFile("test_pcd_" + std::to_string( voxel_size * 100 ) + "_cm.pcd", *cloud);
    pcl::io::savePCDFile("/home/nc_5cm.pcd", *cloud);
    // cloud->points.clear();mv
    // std::cout << "points.clear: " << cloud->size() << std::endl;
  }

  return (0);
}
