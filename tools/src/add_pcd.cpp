
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



void read_pcd_file(const std::string filename, pcxyz_type::Ptr cloud)
{
	auto pcd = filename;
	try
	{
		pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *cloud);
	}
	catch (const std::exception &e)
	{
		PCL_ERROR("can not load pcd file! :  %s", pcd.c_str());
		std::cerr << e.what() << '\n';
	}

}

void filter_sth(pcxyz_type::Ptr & cloud_in_src)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;	
    // downsample clouds
    pcl::VoxelGrid<pcl::PointXYZ> vg;

	// pcxyz_type::Ptr cloud_out(new pcxyz_type);

	// build the filter
	outrem.setInputCloud(cloud_in_src);
	outrem.setRadiusSearch(2);
	outrem.setMinNeighborsInRadius(2);
	// apply filter
	outrem.filter(*cloud_in_src);

	vg.setInputCloud(cloud_in_src);
	vg.setLeafSize(0.1f, 0.1f, 0.1f);
	vg.filter(*cloud_in_src);
}

int main(int argc, char **argv)
{
	// pcxyz_type::Ptr cloud_1(new pcxyz_type);
	// pcxyz_type::Ptr cloud_2(new pcxyz_type);
	// pcxyz_type::Ptr cloud_3(new pcxyz_type);

	// read_pcd_file("all.pcd", cloud_1);
	// filter_sth(cloud_1);

	// read_pcd_file("/home/map/region2/saveMap.pcd", cloud_2);

	// *cloud_3 = *cloud_1 + *cloud_2;
	//     cout << "cloud_3 size " << cloud_3->size() << endl;
	// filter_sth(cloud_3);
	//     cout << "cloud_3 size " << cloud_3->size() << endl;

    // pcl::io::savePCDFileASCII("ok.pcd", *cloud_3 );


	// return 0;


	std::string work_dir = "/home/lj/";
	std::cout << "Your work dir is : " << work_dir << std::endl;

	// 这个读取的顺序是对的
	vector<string> pcd_file;
	scan_dir_get_filename(work_dir, pcd_file);

	pcxyz_type::Ptr one_cloud(new pcxyz_type);
	pcxyz_type::Ptr all_cloud(new pcxyz_type);

	for (int i = 0; i < pcd_file.size()  ; i ++ )
	{
		read_pcd_file(pcd_file[i], one_cloud);
		*all_cloud += *one_cloud;
	    cout << "all_cloud size " << all_cloud->size() << endl;
	}
	    cout << "all_cloud size " << all_cloud->size() << endl;

	// filter_sth(all_cloud);

    pcl::io::savePCDFileASCII("/home/map/saveMap.pcd", *all_cloud);
	    cout << "all_cloud size " << all_cloud->size() << endl;

	cout << "--cnts_ ----DONE----- " << endl;

	return 0;
}
