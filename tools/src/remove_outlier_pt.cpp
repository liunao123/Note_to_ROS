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

int main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/map/region5-51/pose_graph/000445/cloud.pcd", *cloud);

	cout << "points sieze is:" << cloud->size() << endl;
	
	//outlier
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	// sor.setInputCloud(cloud);
	// sor.setMeanK(5);   //设置在进行统计时考虑查询点邻近点数
	// sor.setStddevMulThresh(1.0);   //设置判断是否为离群点的阈值，如果一个点的距离超出平均距离一个标准差以上，则该点被标记为离群点，并将被移除。
	// sor.filter(*cloud_filtered);

    pcl::RadiusOutlierRemoval< pcl::PointXYZ > outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.2);
	outrem.setMinNeighborsInRadius(10);
	// apply filter
	outrem.filter(*cloud_filtered);

	cout << "points sieze is:" << cloud_filtered->size() << endl;

	//---------------------------------显示---------------------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);//xmin, ymin, xmax, ymax,取值范围0-1
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green0(cloud, 0, 225, 0);
	viewer->addPointCloud(cloud, green0, "cloud", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green1(cloud_filtered, 0, 225, 0);
	viewer->addPointCloud(cloud_filtered, green1, "cloud_filtered", v2);
	
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	system("pause");
	return 0;
}

