#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

pcl::VoxelGrid<pcl::PointXYZ> vg;

void filter_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in_src)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;	
  // downsample clouds
  pcl::VoxelGrid<pcl::PointXYZ> vg;

	// pcxyz_type::Ptr cloud_out(new pcxyz_type);

	// build the filter
	outrem.setInputCloud(cloud_in_src);
	outrem.setRadiusSearch(5);
	outrem.setMinNeighborsInRadius(2);
	// apply filter
	outrem.filter(*cloud_in_src);

	vg.setInputCloud(cloud_in_src);
	vg.setLeafSize(0.1f, 0.1f, 0.1f);
	vg.filter(*cloud_in_src);
}

int main()
{
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

  std::string  in_pc = "/home/map/icp/_3_scan1.pcd";
  std::string out_pc = "/home/map/icp/_3_scan2.pcd";

  std::cout << "in_pc " << in_pc << std::endl;
  std::cout << "out_pc " << out_pc << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(in_pc, *cloud_source) == -1)
  {
    PCL_ERROR("Couldn't read file cloud_source.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(out_pc, *cloud_target) == -1)
  {
    PCL_ERROR("Couldn't read file cloud_target.pcd \n");
    return (-1);
  }

  double icp_tf_epsilon_ = 0.000000005;
  // double icp_tf_epsilon_ = 0.001;
  double icp_corr_dist_ = 10;
  int icp_iterations_ = 200;

  double OutlierRejectionThreshold_ = 0.3;
  double EuclideanFitnessEpsilon_ = 0.25;

  // filter_pointcloud(cloud_source);
  // filter_pointcloud(cloud_target);

  gicp.setInputSource(cloud_source);
  gicp.setInputTarget(cloud_target);
  std::cout << "cloud_source:" << cloud_source->size() << std::endl;
  std::cout << "cloud_target:" << cloud_target->size() << std::endl;

  gicp.setTransformationEpsilon(icp_tf_epsilon_);
  gicp.setMaxCorrespondenceDistance(icp_corr_dist_);
  gicp.setMaximumIterations(icp_iterations_);
  // gicp.setRANSACIterations(500);
  gicp.setRANSACIterations(10);

  // gicp.setEuclideanFitnessEpsilon (0.25);
  gicp.setEuclideanFitnessEpsilon( EuclideanFitnessEpsilon_ );
  gicp.setRANSACOutlierRejectionThreshold(OutlierRejectionThreshold_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  gicp.align(*Final);
  std::cout << "Final:" << Final->size() << std::endl
            << std::endl;
  std::cout << "icp has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
  std::cout << gicp.getTransformationEpsilon() << " " << gicp.getMaxCorrespondenceDistance() << " " << gicp.getMaximumIterations() << std::endl;

  Final->height = 1;
  Final->width = Final->size();

  // pcl::visualization::PCLVisualizer visu("Alignment");
  // visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
  // visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
  // visu.spin();

  pcl::io::savePCDFileASCII("/opt/csg/slam/navs/s_align.pcd", *Final);

  return (0);
}
