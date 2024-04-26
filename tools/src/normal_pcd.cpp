#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>

int main(int argc, char **argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "cloud : " << cloud->size() << " points to pcd." << std::endl;


  double mean_x = 0 , mean_y = 0 , mean_z = 0 ;

  for (int i = 0; i < cloud->size(); ++i) {
    mean_x += cloud->points[i].x;
    mean_y += cloud->points[i].y;
    mean_z += cloud->points[i].z;

  }

  mean_x = mean_x / cloud->size() ;
  mean_y = mean_y / cloud->size() ;
  mean_z = mean_z / cloud->size() ;
  std::cout << " mean_x " << mean_x << std::endl;
  std::cout << " mean_y " << mean_y << std::endl;
  std::cout << " mean_z " << mean_z << std::endl;

 
  for (int i = 0; i < cloud->size(); ++i) {
     cloud->points[i].x -= mean_x  ;
     cloud->points[i].y -= mean_y  ;
     cloud->points[i].z -= mean_z  ;
  }

  cloud->is_dense = false;
  cloud->width = cloud->points.size();
  cloud->height = 1;
  pcl::io::savePCDFileASCII("/home/output_cloud.pcd", *cloud);

  printf("44 ok");
  std::cerr << "Saved <output_cloud> : " << cloud->width << " points to pcd." << std::endl;
  return 1;
}
