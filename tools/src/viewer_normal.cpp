/*****************************************************************//**
* \file   PCLFeatureNormalmain.cpp
* \brief  
*
* \author YZS
* \date   January 2025
*********************************************************************/
#include<iostream>
#include <vector>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>//法向量计算头文件
using namespace std;
void PCLNoramls()
{
       //加载点云数据
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
      std::string fileName = "/opt/csg/slam/navs/6s_normal_1cm.pcd";
       pcl::io::load(fileName, *cloud);
       std::cout << "Cloud Size:" << cloud->points.size() << std::endl;
       //法向量估计器对象
       pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
       ne.setInputCloud(cloud);
       //创建搜索方法KDtree
       pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZRGB>());
       ne.setSearchMethod(tree);
       //法向量结果
       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new  pcl::PointCloud<pcl::Normal>);
       //设置领域搜索半径
       ne.setRadiusSearch(0.02);
       //特征计算
       ne.compute(*cloud_normals);
       std::cout << "filter Cloud Size:" << cloud_normals->points.size() << std::endl;
       //法向量结果可视化
   // PCLVisualizer对象
       pcl::visualization::PCLVisualizer viewer("FeatureVIS");
       //创建左右窗口的ID v1和v2
       int v1(0);
       int v2(1);
       //设置V1窗口尺寸和背景颜色
       viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
       viewer.setBackgroundColor(0, 0, 0, v1);
       //设置V2窗口尺寸和背景颜色
       viewer.createViewPort(0.5, 0.0, 1, 1, v2);
       viewer.setBackgroundColor(0.1, 0.1, 0.1, v2);
       // 添加2d文字标签
       viewer.addText("v1", 10, 10, 20, 1, 0, 0, "Txtv1", v1);
       viewer.addText("v2", 10, 10, 20, 0, 1, 0, "Txtv2", v2);
       //设置cloud1的渲染属性，点云的ID和指定可视化窗口v1
       viewer.addPointCloud(cloud, "cloud1", v1);
       viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,  2, "cloud1");
       //设置法向量结果的渲染到V2窗口中
       viewer.addPointCloud(cloud, "cloud2", v2);
       viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud,cloud_normals,3,0.1f,  "cloud2n", v2);
       
       viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,  2, "cloud2");
       // 可视化循环主体
       while (!viewer.wasStopped())
       {
              viewer.spinOnce();
       }
}
int main(int argc,char *argv[])
{
       PCLNoramls();
  std::cout<<"Hello PCL World!"<<std::endl;
  std::system("pause");
  return 0;
}