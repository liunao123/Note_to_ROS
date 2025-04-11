#include "pack.h"
int main(int argc, char **argv)
{
    std::string pcd_file = "/home/liunao/Kalibr/v1_20241118/temp/2ds.pcd";
    double SearchRadius = 0.1;
    if (argc == 3)
    {
        std::cout << "you should specify pcd file  . " << std::endl;
        pcd_file = argv[1];
        SearchRadius = std::stod(argv[2]) ;
        // return -1;
    }

    std::cout << "pcd_file: " << pcd_file << std::endl;
    std::cout << "SearchRadius: " << SearchRadius << std::endl;

    // PointCloudTPtr cloud(new PointCloudT());
    // // std::cout << "pcd_file : " << pcd_file << std::endl;
    // if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1)
    // {
    //     PCL_ERROR("Could not read file\n");
    // }
    // std::cout << "cloud points size : " << cloud->size() << std::endl;

    // 1
    // pack p;
    // p.setInputCloud(cloud);

    // 2
    pack p(pcd_file);
    p.setSearchRadius( SearchRadius );
    // p.setSearchRadius(0.15);
    p.setOrthogonalityTHRESHOLD(0.15);
    p.run();

    PointCloudTPtr out_cloud(new PointCloudT());
    p.getNormalCloud(out_cloud);
    pcl::io::savePCDFile("/opt/csg/slam/navs/cloud_normal_.pcd", *out_cloud);
    p.getOutlierCloud(out_cloud);
    pcl::io::savePCDFile("/opt/csg/slam/navs/cloud_outlier_.pcd", *out_cloud);


}
