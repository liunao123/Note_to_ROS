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
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointType;
// typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointTypeCloud;

pcl::VoxelGrid<PointType> vg;

void filter_pointcloud(PointTypeCloud::Ptr & cloud_in_src)
{
	pcl::RadiusOutlierRemoval<PointType> outrem;	
  // downsample clouds
  pcl::VoxelGrid<PointType> vg;

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

void scalePointCloud(PointTypeCloud::Ptr & cloud, float scale_factor = 10) {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x *= scale_factor;
        cloud->points[i].y *= scale_factor;
        cloud->points[i].z *= scale_factor;
    }
}

int main()
{
  // pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  pcl::IterativeClosestPoint<PointType, PointType> gicp;

  PointTypeCloud::Ptr cloud_source(new PointTypeCloud);
  PointTypeCloud::Ptr cloud_target(new PointTypeCloud);

  std::string  in_pc = "/opt/csg/slam/navs/test_d435i/57cm.pcd";
  std::string out_pc = "/opt/csg/slam/navs/test_d435i/60cm.pcd";

  std::cout << "in_pc " << in_pc << std::endl;
  std::cout << "out_pc " << out_pc << std::endl;
  if (pcl::io::loadPCDFile<PointType>(in_pc, *cloud_source) == -1)
  {
    PCL_ERROR("Couldn't read file cloud_source.pcd \n");
    return (-1);
  }


  if (pcl::io::loadPCDFile<PointType>(out_pc, *cloud_target) == -1)
  {
    PCL_ERROR("Couldn't read file cloud_target.pcd \n");
    return (-1);
  }
  // scalePointCloud(cloud_source);
  // scalePointCloud(cloud_target);
  pcl::io::savePCDFileASCII("/opt/csg/slam/navs/test_d435i/cloud_source.pcd", *cloud_source);
  pcl::io::savePCDFileASCII("/opt/csg/slam/navs/test_d435i/cloud_target.pcd", *cloud_target);

  double icp_tf_epsilon_ = 1e-5;
  // double icp_corr_dist_ = 0.1;
  double icp_corr_dist_ = 0.1;
  int icp_iterations_ = 200;

  double OutlierRejectionThreshold_ = 0.02;
  double EuclideanFitnessEpsilon_ = 0.02;

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
  // gicp.setEuclideanFitnessEpsilon( EuclideanFitnessEpsilon_ );
  // gicp.setRANSACOutlierRejectionThreshold(OutlierRejectionThreshold_);

  PointTypeCloud::Ptr Final(new PointTypeCloud);
  gicp.align(*Final);
  std::cout << "Final:" << Final->size() << std::endl
            << std::endl;
  std::cout << "icp has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
  std::cout << gicp.getTransformationEpsilon() << " " << gicp.getMaxCorrespondenceDistance() << " " << gicp.getMaximumIterations() << std::endl;

  std::cout << "-----------------"  << std::endl;
	std::cout << gicp.getFinalTransformation() << std::endl;

  Final->height = 1;
  Final->width = Final->size();

  // pcl::visualization::PCLVisualizer visu("Alignment");
  // visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
  // visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
  // visu.spin();

  pcl::io::savePCDFileASCII("/opt/csg/slam/navs/test_d435i/s_align.pcd", *Final);

  float scale_factor = 100.0f; // 扩大 100 倍
  scalePointCloud(cloud_source, scale_factor);

  return (0);
}
