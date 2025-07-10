#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <iostream>
#include <fstream>

#include <omp.h>
#include <ctime>
#include <vector>
#include <string>
#include <algorithm>
 
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
 
#include <sys/stat.h>
 


typedef pcl::PointXYZI pt;
 

void bin2pcd(const std::string &in_file, const std::string& out_file)
{
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Couldn't read in_file: " << in_file << std::endl;
    }

    pcl::PointCloud<pt>::Ptr points (new pcl::PointCloud<pt>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pt point;
        input.read((char *) &point.x, sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));

        if (point.intensity > 3)
        {
          points->push_back(point);
        }
    }
    input.close();

    pcl::io::savePCDFileASCII(out_file, *points);
    std::cerr << "DONE: " << out_file << std::endl;

}

void pcd2bin(const std::string &in_file, const std::string &out_file)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1)
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return; // (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from "
              << in_file
              << " with the following fields: "
              << std::endl;

    std::ofstream bin_file(out_file.c_str(), std::ios::out | std::ios::binary);

    for (int j = 0; j < cloud->size(); j++)
    {
        bin_file.write((char *)&cloud->at(j).x, sizeof(float));
        bin_file.write((char *)&cloud->at(j).y, sizeof(float));
        bin_file.write((char *)&cloud->at(j).z, sizeof(float));
        bin_file.write((char *)&cloud->at(j).intensity, sizeof(float));
    }
    bin_file.close();
    std::cerr << "DONE: " << out_file << std::endl;

}




int main() 
{
    std::cerr << "ssssssssssssssssssssss: " << std::endl;
    // std::string binfile = "/opt/csg/slam/navs/oneformer3d-main/data/s3dis/points/Area_1_office_10.bin";
    std::string binfile = "/opt/csg/slam/navs/scene_03400_0.bin";
    std::string pcdfile = "/opt/csg/slam/navs/scene_03400_0_mask.pcd";
    bin2pcd(binfile, pcdfile);

    // pcd2bin(pcdfile, binfile);

    return 1;
}