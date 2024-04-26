
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <algorithm>

#include <stdlib.h>
#include <stdio.h>

// #include <ros/ros.h>

#include <sys/types.h>
#include <dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> pcxyz_type;

void filter_sth(pcxyz_type::Ptr & cloud_in_src)
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
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_in_src);
}

void read_pcd_file(const std::string filename, pcxyz_type::Ptr cloud)
{
	auto pcd = filename + ".pcd";
	try
	{
		pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *cloud);
	}
	catch (const std::exception &e)
	{
		PCL_ERROR("can not load pcd file! :  %s", pcd.c_str());
		std::cerr << e.what() << '\n';
	}
	filter_sth(cloud);
}



int main(int argc, char **argv)
{
	std::string work_dir = "/home/bag/livox_hk/cab/data/";
	std::cout << "Your work dir is : " << work_dir << std::endl;
	for (int i = 0; i < 4; i++)
	{
		std::string pcd = work_dir + std::to_string(i);
		std::cout << "now pcd is : " << pcd << std::endl;

    	pcxyz_type::Ptr target_cloud(new pcxyz_type);
    	pcxyz_type::Ptr output_cloud(new pcxyz_type);
		read_pcd_file(pcd, target_cloud);

		for (uint j = 0; j < target_cloud->size(); ++j)
		{
			if (std::fabs(target_cloud->points[j].y) < 10.0  &&    target_cloud->points[j].x < 15.0  &&    target_cloud->points[j].x > 3.0   )
			{
				output_cloud->points.push_back(target_cloud->points[j]);
			}
		}
		output_cloud->is_dense = false;
		output_cloud->width = output_cloud->points.size();
		output_cloud->height = 1;
		pcl::io::savePCDFileASCII( work_dir + std::to_string(i) + "_box.pcd", *output_cloud);
		printf("44 ok");
	}


	return 0;
}
