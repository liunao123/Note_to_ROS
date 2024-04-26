
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <algorithm>

#include <stdlib.h>
#include <stdio.h>

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

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

void scan_dir_get_filename(string path, vector<string> &filenames)
{
	struct dirent **entry_list;
	int count;
	int i;

	count = scandir(path.c_str(), &entry_list, 0, alphasort);
	if (count < 0)
	{
		perror("scandir");
	}

	for (i = 0; i < count; i++)
	{
		struct dirent *entry;
		entry = entry_list[i];
		// printf("%s\n", entry->d_name);
		// 跳过 ./ 和 ../ 两个目录
		if (i < 2)
		{
			continue;
		}
		filenames.push_back(path + std::string(entry->d_name));
		// std::cout << "filenames  is : " << filenames.back() << std::endl;

		free(entry);
	}
	std::cout << "filenames size is : " << filenames.size() << std::endl;

	free(entry_list);
}


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
	vg.setLeafSize(0.1f, 0.1f, 0.1f);
	vg.filter(*cloud_in_src);
}

void read_pcd_file(const std::string filename, pcxyz_type::Ptr cloud)
{
	auto pcd = filename + "/cloud.pcd";
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

double do_gicp(pcxyz_type::Ptr head_cloud, pcxyz_type::Ptr tail_cloud, pcxyz_type::Ptr cloud_aligned)
{
	gicp.setInputSource(head_cloud);
	gicp.setInputTarget(tail_cloud);
	// Set GICP parameters
	gicp.setMaximumIterations(200);
	gicp.setTransformationEpsilon(0.0003);
    gicp.setMaxCorrespondenceDistance(5.0);
	// gicp.setEuclideanFitnessEpsilon(1e-6);

    //   <param name="gicp_tf_epsilon"    type="double" value="" />
    //   <param name="gicp_corr_dist"     type="double" value="5.0" />
    //   <param name="gicp_iterations"    type="int"    value="200" />

	// Align source to target
	gicp.align(*cloud_aligned);

	// std::cout << "Transformation matrix:" << std::endl;
	// std::cout << gicp.getFinalTransformation() << std::endl;

	if (!gicp.hasConverged())
	{
		printf("icp isn't coverged");
		return 1000.0;
	}
	// printf("score %f", gicp.getFitnessScore());
	return gicp.getFitnessScore();
}

int main(int argc, char **argv)
{
	std::string work_dir = "/home/map/region1-6/pose_graph/";
	std::cout << "Your work dir is : " << work_dir << std::endl;

	// 这个读取的顺序是对的
	vector<string> pcd_file;
	scan_dir_get_filename(work_dir, pcd_file);

	int head_size = 50;
	int tail_size = 50;
	int step = 3;

	pcxyz_type::Ptr head_cloud(new pcxyz_type);
	pcxyz_type::Ptr tail_cloud(new pcxyz_type);
    pcxyz_type::Ptr cloud_aligned(new pcxyz_type);

	int cnts = 0;
	std::vector< pcxyz_type::Ptr > tail_pcd_vec( pcd_file.size() );
	cout << " " << tail_pcd_vec.size() << endl;
	bool first_flag = true;

	for (int i = 10; i < head_size; i = i + step)
	{
		read_pcd_file(pcd_file[i], head_cloud);
		for (int j = pcd_file.size() - tail_size; j < pcd_file.size() - 1; j = j + 5)
		{
			if( first_flag )
			{
			    read_pcd_file(pcd_file[j], tail_cloud);
				tail_pcd_vec[j] = tail_cloud   ;
			}
			else
			{
				tail_cloud = tail_pcd_vec[j];
			}

			auto fitness = do_gicp(head_cloud, tail_cloud , cloud_aligned);
			cout << i << " " << j << " " << fitness << endl;

			if (fitness < 2.0)
			{
				cnts++;
				std::string target_file = "/home/map/region1-6/"  +  std::to_string(cnts) + ".pcd";
				std::string aligned_file = "/home/map/region1-6/"  +  std::to_string(cnts) + "_aligned.pcd";
				pcl::io::savePCDFileASCII(target_file, *tail_cloud);
				pcl::io::savePCDFileASCII(aligned_file, *cloud_aligned);
            	cout << "cnts_   " << cnts << endl;
			}
			
		}
		first_flag = false;
	}

	cout << "--cnts_ ----DONE----- " << endl;

	return 0;
}
