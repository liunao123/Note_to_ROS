#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{

  if (argc != 3)
    return -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }

  pcl::PointCloud<pcl::PointXYZ> output_cloud;

  // for (uint i = 0; i < target_cloud->size(); ++i) {
  //       if( std::fabs( target_cloud->points[i].y ) < 5.0 )
  //       {
  //         output_cloud.points.push_back(target_cloud->points[i]);
  //       }
  //     }
  // output_cloud.is_dense = false;
  // output_cloud.width = output_cloud.points.size();
  // output_cloud.height = 1;
  // pcl::io::savePCDFileASCII("test_pcd_output_cloud.pcd", output_cloud);
  // printf("44 ok");
  // return 1;



  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterTempMap;
  const float voxel_size = std::stof( argv[2] );

  for (size_t i = 0; i < 1; i++)
  {  
  downSizeFilterTempMap.setLeafSize(voxel_size + double(i) / 20 , voxel_size + double(i) / 20 ,  voxel_size + double(i) / 20 );
  downSizeFilterTempMap.setInputCloud(target_cloud);
  downSizeFilterTempMap.filter(*cloud);
  pcl::io::savePCDFile("test_pcd_" + std::to_string( int( 100 * (voxel_size + double(i) / 20) ) ) + "_cm.pcd", *cloud);
  // pcl::io::savePCDFile("/home/pcd_10_cm.pcd", *cloud);
  std::cerr << "Saved voxel after: " << cloud->size() << " data points to test_pcd.pcd." << std::endl;

  }

  return (0);
}
