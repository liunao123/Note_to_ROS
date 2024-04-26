#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>

int main(int argc, char **argv)
{
  Eigen::Vector4f pt = {-1,0,0,0};
	Eigen::Vector4f line_pt_1 = { 3.739   , 21.166 ,  0,0 };
	Eigen::Vector4f line_pt_2 = { 142.486 , 18.066 ,  0,0 };
	Eigen::Vector4f line_dir= line_pt_2 - line_pt_1;

  double dis_sqr = pcl::sqrPointToLineDistance(pt, line_pt_1, line_dir);
  printf("27 dis is %f \n", dis_sqr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cerr << "cloud : " << cloud->size() << " points to pcd." << std::endl;

  pcl::PointCloud<pcl::PointXYZ> output_cloud_car;
  pcl::PointCloud<pcl::PointXYZ> output_cloud_remove_car;

  const float half_kd_width = 3.5; 

  for (uint i = 0; i < cloud->size(); ++i) {

    // pt = Eigen::Vector4f (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 0);
    // dis_sqr = pcl::sqrPointToLineDistance(pt, line_pt_1, line_dir);
    // if (dis_sqr < half_kd_width * half_kd_width && cloud->points[i].z > -0.30)
    //   output_cloud_car.points.push_back(cloud->points[i]); // 车
    // else
    //   output_cloud_remove_car.points.push_back(cloud->points[i]); // 去掉 车

    if(std::fabs(cloud->points[i].x) < 20.0 && cloud->points[i].y > -3.0 &&  cloud->points[i].x < 10.0 )
      output_cloud_remove_car.points.push_back(cloud->points[i]); // 去掉 车

  }


  // output_cloud_car.is_dense = false;
  // output_cloud_car.width = output_cloud_car.points.size();
  // output_cloud_car.height = 1;
  // pcl::io::savePCDFileASCII("/home/_che.pcd", output_cloud_car);
  // std::cerr << "Saved <car>: " << output_cloud_car.size() << " points to pcd." << std::endl;

  output_cloud_remove_car.is_dense = false;
  output_cloud_remove_car.width = output_cloud_remove_car.points.size();
  output_cloud_remove_car.height = 1;
  pcl::io::savePCDFileASCII("/home/remove_che.pcd", output_cloud_remove_car);

  printf("44 ok");
  std::cerr << "Saved <removed car> : " << output_cloud_remove_car.size() << " points to pcd." << std::endl;
  return 1;
}
