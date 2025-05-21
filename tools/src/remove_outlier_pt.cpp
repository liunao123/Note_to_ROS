#include <iostream>
#include <vector>
#include <ctime>
//---------------------------------------------
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h> //obj读取头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
//-法向显示错误：no override found for vtkActor--
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
//outlier
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;

int main(int argc, char **argv)
{
    std::string filename = "/opt/csg/slam/navs/6_shadows.pcd";
    if (argc == 2)
    {
      filename = argv[1];
    }
    else
    {
      std::cout << "you can specify pcd file . like: " << std::endl;
      std::cout << " ./remove_outlier_pt /home/1.pcd " << std::endl;
    }

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud);

	cout << "points sieze is:" << cloud->size() << endl;
	
	//outlier
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	// sor.setInputCloud(cloud);
	// sor.setMeanK(5);   //设置在进行统计时考虑查询点邻近点数
	// sor.setStddevMulThresh(1.0);   //设置判断是否为离群点的阈值，如果一个点的距离超出平均距离一个标准差以上，则该点被标记为离群点，并将被移除。
	// sor.filter(*cloud_filtered);

	// 去除地铁立柱之间的噪点
	// 在一个点0.15m范围内找到至少15个点，才会保留这个点
    pcl::RadiusOutlierRemoval< pcl::PointXYZI > outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.15);
	outrem.setMinNeighborsInRadius(15);
	// apply filter
	outrem.filter(*cloud_filtered);

	cout << "points sieze is:" << cloud_filtered->size() << endl;

        pcl::io::savePCDFileASCII("/opt/csg/slam/navs/22.pcd", *cloud_filtered);
 
	return 0;
}

