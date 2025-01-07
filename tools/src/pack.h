#ifndef PACK_H_
#define PACK_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense> // Include the Eigen library for matrix operations

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud< PointT >  PointCloudT;
typedef pcl::PointCloud< PointT >::Ptr PointCloudTPtr;

class pack
{
private:
    // 定义的时候直接初始化，要不然得reset
    PointCloudTPtr cloud_{new PointCloudT()}; 
    PointCloudTPtr cloud_normal_{new PointCloudT()}; 
    PointCloudTPtr cloud_outlier_{new PointCloudT()};
    double SearchRadius_ = 0.1;
    double OrthogonalityTHRESHOLD_ = 0.2;
public:
    void setInputCloud(const PointCloudTPtr& cloud);
    void getNormalCloud(PointCloudTPtr& cloud);
    void getOutlierCloud(PointCloudTPtr& cloud);
    void setSearchRadius(const double & value);
    void setOrthogonalityTHRESHOLD(const double & value);
    int run();
    pack(const std::string pcd_file);
    pack();
    ~pack();
};

#endif // PACK_H_
