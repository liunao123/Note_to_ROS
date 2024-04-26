#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr my_table_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr my_table_voxel_process_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("/home/liunao/qt/temp_pcd_jiangyangbeilu/1680108028722012.pcd",*my_table_voxel_process_cloud);
	//由于原有的table点云数据量太大，进行降采样

	// pcl::VoxelGrid<pcl::PointXYZ> my_voxel_filter;
	// my_voxel_filter.setInputCloud(my_table_cloud);
	// my_voxel_filter.setLeafSize(0.1f,0.1f,0.1f);
	// my_voxel_filter.filter(*my_table_voxel_process_cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr indices(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> sacseg;
	sacseg.setOptimizeCoefficients(true);
	sacseg.setModelType(pcl::SACMODEL_PLANE);
	sacseg.setMethodType(pcl::SAC_RANSAC);
	sacseg.setMaxIterations(100);
	sacseg.setDistanceThreshold(0.001);
	sacseg.setInputCloud(my_table_voxel_process_cloud);
	sacseg.segment(*indices,*coefficients);

	if (indices->indices.size() == 0)
	{
		std::cerr<<"Could not estimate a planar model for the given dataset."<<std::endl;
	}
	pcl::ExtractIndices<pcl::PointXYZ> my_extract_indices;
	my_extract_indices.setInputCloud(my_table_voxel_process_cloud);
	my_extract_indices.setIndices(indices);
	my_extract_indices.setNegative(false);
	my_extract_indices.filter(*cloud_p);
    PCL_ERROR("cloud_p %ld \n", cloud_p->points.size());

    (*cloud_p).is_dense = false;
    (*cloud_p).width = (*cloud_p).points.size();
    (*cloud_p).height = 1;
    pcl::io::savePCDFileASCII("/home/liunao/qt/temp_pcd_jiangyangbeilu/1680108028722012_plan.pcd", *cloud_p);

	pcl::visualization::PCLVisualizer my_viewer1;
	pcl::visualization::PCLVisualizer my_viewer2; 
	// my_viewer1.addPointCloud(my_table_cloud);
	my_viewer2.addPointCloud(cloud_p);
    

	while(!my_viewer1.wasStopped())
	{
		// my_viewer1.spinOnce(100);
		my_viewer1.spinOnce(100);
	}


    return 1;
}