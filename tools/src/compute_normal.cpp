#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <ctime>//c++程序计时头文件
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
clock_t start_time, end_time;

//计算点云的n个最大曲率点——函数声明
void MaxCurvaturePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

//曲率计算结构体
typedef struct PCURVATURE{
	//POINT3F cPoint;
	int index;
	float curvature;
}PCURVATURE;

int main(int argc, char **argv)
{
    std::string pcd_file = "/home/liunao/Kalibr/v1_20241118/temp/2ds.pcd";
    if (argc == 2)
    {
        std::cout << "you should specify pcd file  . " << std::endl;
        pcd_file = argv[1];
        // return -1;
    }

    std::cout << "pcd_file: " << pcd_file << std::endl;


	//读取点云数据
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read( pcd_file , *cloud);

	start_time = clock();//程序开始计时
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_MaxCurvature(new pcl::PointCloud<pcl::PointXYZ>);

	MaxCurvaturePoints(cloud, 200000, cloud_MaxCurvature);//点云的5000个最大曲率点
	
    end_time = clock();//程序结束用时
	double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	cout << "Total time:" << endtime << "s" << endl;//s为单位
	cout << "Total time:" << endtime * 1000 << "ms" << endl;//ms为单位

	//可视化
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_MaxCurvature, 255, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_MaxCurvature, color, "cloud_MaxCurvature");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

//计算点云的n个最大曲率点——函数实现
void MaxCurvaturePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	//点云法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_in);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normals_curvatures(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree_normals_curvatures);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	// ne.setRadiusSearch(0.1);
	ne.setRadiusSearch(0.2);
	// ne.setKSearch(20);
	ne.compute(*cloud_normals);//计算法线

	//计算点云曲率
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud_in);
	pc.setInputNormals(cloud_normals);
	pc.setSearchMethod(tree_normals_curvatures);
	// pc.setRadiusSearch(0.03);//设置搜索半径，可设为0.01
	pc.setRadiusSearch(0.2);//设置搜索半径，可设为0.01
	// pc.setKSearch(10);
	pc.compute(*cloud_curvatures);

	//计算点云的高斯曲率，获取曲率集
	std::vector<PCURVATURE> allCurvates;
	pcl::PointXYZ tempPoint;
	float curvature = 0.0;
	PCURVATURE pv;
	tempPoint.x = tempPoint.y = tempPoint.z = 0.0;
	for (int i = 0; i < cloud_curvatures->size(); i++){
		//平均曲率
		curvature = ((*cloud_curvatures)[i].pc1 + (*cloud_curvatures)[i].pc2) / 2;
		//高斯曲率
		//curvature = (*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2;
		//pv.cPoint = tempPoint;
		pv.index = i;
		pv.curvature = curvature;
		allCurvates.insert(allCurvates.end(), pv);
	}
	//获取曲率最大的前n个点
	PCURVATURE temp;
	int maxIndex = 0;
	int count = 0;
	for (int i = 0; i < allCurvates.size(); i++){
		float maxCurvature = -99999;
		for (int j = i + 1; j< allCurvates.size(); j++){
			if (maxCurvature<allCurvates[j].curvature){
				maxCurvature = allCurvates[j].curvature;
				maxIndex = j;
			}
		}
		if (maxCurvature>allCurvates[i].curvature){
			temp = allCurvates[maxIndex];
			allCurvates[maxIndex] = allCurvates[i];
			allCurvates[i] = temp;
			count++;
		}
		if (count>n){
			break;
		}
	}
	//提取最大曲率点
	for (int i = 0; i < n; i++)
	{
		int indexP = allCurvates[i].index;
		cloud_out->push_back(cloud_in->points[indexP]);
	}
}
