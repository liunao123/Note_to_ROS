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

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudT;

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cout << "you should specify pcd file and resolution_ "  << std::endl;
    return -1;
  }

  std::string filename = argv[1];
  std::cout << "filename: " << filename << std::endl;
  
  float resolution_ = 0.1f;
  resolution_ = std::stof(argv[2]);
  std::cout << "resolution_: " << resolution_ << std::endl;

  PointCloudT::Ptr  cloud(new PointCloudT);
  PointCloudT::Ptr cloud_out(new PointCloudT);

  if (pcl::io::loadPCDFile(filename, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }
  std::cout << "pts: " <<  cloud->size() << std::endl;

    // generate octree
    typename pcl::octree::OctreePointCloud<PointType> octree(resolution_);
    // add point cloud to octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    // get points where grid is occupied
    typename pcl::octree::OctreePointCloud<PointType>::AlignedPointTVector point_vec;
    octree.getOccupiedVoxelCenters(point_vec);
    // put points into point cloud
    cloud_out->width = point_vec.size();
    cloud_out->height = 1;

  std::cout << "point_vec: " <<  point_vec.size() << std::endl;

    for (int i = 0; i < point_vec.size(); i++) {
      cloud_out->push_back(point_vec[i]);
    }

  std::cout << "cloud_out: " <<  cloud_out->size() << std::endl;
    // 4. 保存结果
    pcl::io::savePCDFile("/opt/csg/slam/navs/voxel_centroids.pcd", *cloud_out);

  pcl::RadiusOutlierRemoval< PointType > ror;	//创建统计滤波器对象
	ror.setInputCloud( cloud_out );		//设置待滤波点云
	ror.setRadiusSearch(0.25);	// 设置查询点的半径范围
	ror.setMinNeighborsInRadius(8);// 设置判断是否为离群点的阈值，即半径内至少包括的点数
	// ror.setNegative(true);//默认false，保存内点；true，保存滤掉的离群点
	ror.filter(*cloud_out);			//执行滤波，保存滤波结果在cloud_filtered
  std::cout << "cloud_out: " <<  cloud_out->size() << std::endl;
    pcl::io::savePCDFile("/opt/csg/slam/navs/voxel_centroids_ror.pcd", *cloud_out);


	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval< PointType > sor;
	sor.setInputCloud(cloud_out);//设置待滤波的点云
	sor.setMeanK(10);//设置在进行统计时考虑查询点邻居点数
	sor.setStddevMulThresh(1.0);//设置判断是否为离群点的阈值
	sor.filter(*cloud_out);//将滤波结果保存在cloud_filtered中
  std::cout << "cloud_out: " <<  cloud_out->size() << std::endl;


    pcl::io::savePCDFile("/opt/csg/slam/navs/voxel_centroids_sor.pcd", *cloud_out);
                        
    return 0;
}
