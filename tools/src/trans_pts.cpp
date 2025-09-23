#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


int main(int argc, char** argv) 
{
	std::string filename = "/home/tyjt/Desktop/Note_to_ROS/tools/1751423145.999886.pcd";
	if (argc == 2)
	{
		filename = argv[1];
	}
    std::cout << "filename: " << filename << std::endl;

     Eigen::Affine3d T_wl = Eigen::Affine3d::Identity();
     T_wl.translation() = Eigen::Vector3d(-1119.411 , 235.629 , 12.621 ) - Eigen::Vector3d( -275000.0223369 , -3479281.54229995, 0.0);
     Eigen::Quaterniond q1( 0.586279, -0.010523, -0.006159, -0.810017  );
     T_wl.rotate(q1);
     std::cout << "T_wl: " << T_wl.matrix()  << std::endl << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud_o) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
     // 对点云进行变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_o, *transformedCloud, T_wl);

    transformedCloud->width = transformedCloud->size();
    transformedCloud->height = 1;
    pcl::io::savePCDFile("/home/tyjt/Desktop/Note_to_ROS/tools/1751423145.999886_gnss.pcd", *transformedCloud);

    return 0;
}
