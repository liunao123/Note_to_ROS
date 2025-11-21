#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
 
int main()
{
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/mnt/nvme0n1p2/data/nongan_m2_o/pcd/1718661190.302_o.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file input_cloud.pcd\n");
        return -1;
    }
    float range = 20.0;
 
    // 定义裁剪框的范围
    double min_x = -range;
    double max_x = range;
    double min_y = -range;
    double max_y = range;
    double min_z = -range;
    double max_z = range;
    std::cout << "Filtered Point Cloud: " << cloud->size() << " points" << std::endl;
 
    // 创建裁剪对象并设置裁剪范围
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
    crop.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
    crop.filter(*cloud);
 
    // 输出裁剪后的点云信息
    std::cout << "Filtered Point Cloud: " << cloud->size() << " points" << std::endl;
 
    return 0;
}