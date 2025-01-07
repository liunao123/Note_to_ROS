#include "pack.h"
 
int main()
{
    std::string pcd_file = "/opt/csg/slam/navs/6_1cm.pcd";

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
    p.setSearchRadius(0.1);
    p.setOrthogonalityTHRESHOLD(0.15);
    p.run();

    PointCloudTPtr out_cloud(new PointCloudT());
    p.getNormalCloud(out_cloud);
    pcl::io::savePCDFile("/opt/csg/slam/navs/6-cloud_normal_.pcd", *out_cloud);
    p.getOutlierCloud(out_cloud);
    pcl::io::savePCDFile("/opt/csg/slam/navs/6-cloud_outlier_.pcd", *out_cloud);
}
