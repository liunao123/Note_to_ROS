    
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


using namespace std;

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
		std::cout << "filenames  is : " << filenames.back() << std::endl;

		free(entry);
	}
	std::cout << "filenames size is : " << filenames.size() << std::endl;

	free(entry_list);
}

int main(int argc, char **argv)
{
	std::string work_dir = "/home/map/icp";
	std::cout << "Your work dir is : " << work_dir << std::endl;
	// 这个读取的顺序是对的
	vector<string> pcd_file;
	scan_dir_get_filename(work_dir, pcd_file);
    
    for (size_t cnts = 0; cnts < pcd_file.size() / 3; cnts ++ )
    {
    std::string name1 = work_dir + "/_" + std::to_string(cnts) + "_scan2.pcd";
    // pcl::io::savePCDFile(name, *scan2);
	// std::cout << "Your work dir is : " << name1 << std::endl;

    std::string name2 = work_dir + "/_" + std::to_string(cnts) + "_unused_result.pcd";
	auto cmd = "pcl_viewer " + name1 + " " + name2;
	std::cout << "cmd is : " << cmd << std::endl << std::endl;

	system( cmd.c_str() );
	std::cout  << std::endl  << std::endl << std::endl;


    }

}