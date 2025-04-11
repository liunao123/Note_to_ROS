#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>
#include <pcl/search/kdtree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <iostream>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

typedef pcl::PointCloud< PointT >::Ptr PointCloudTPtr;

double calculateLocalDensity(const PointT &point, const PointCloudT::Ptr &cloud, double search_radius)
{
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if (kdtree.radiusSearch(point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    return static_cast<double>(pointIdxRadiusSearch.size());
  }
  else
  {
    return 0.0;
  }
}

double calculateAdaptiveRadius(double local_density, double base_radius)
{
  // Example logic: inverse relationship between density and radius
  return base_radius / (1.0 + local_density);
}

void estimateNormalsWithAdaptiveRadius(const PointCloudT::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>());
  kdtree->setInputCloud(cloud);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);


  for (size_t i = 0; i < cloud->points.size(); ++i)
  {

    double base_radius = 0.05; // Base radius for density calculation

    const PointT &point = cloud->points[i];

    // double local_density = calculateLocalDensity(point, cloud, base_radius);
    // double adaptive_radius = calculateAdaptiveRadius(local_density, base_radius);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    while (kdtree->radiusSearch(point, base_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 50)
    {
      base_radius = base_radius + 0.02;
    }

    if (1)
    {
      pcl::PointCloud<PointT>::Ptr neighborhood(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*cloud, pointIdxRadiusSearch, *neighborhood);

      std::cout << "base_radius : " << base_radius << std::endl;
      std::cout << "pointIdxRadiusSearch.size: " << pointIdxRadiusSearch.size() << std::endl;

      pcl::NormalEstimation<PointT, pcl::Normal> ne_local;

      ne_local.setInputCloud(neighborhood);
      ne_local.setSearchMethod(tree);
      ne_local.setRadiusSearch(2 * base_radius);

      pcl::PointCloud<pcl::Normal>::Ptr local_normals(new pcl::PointCloud<pcl::Normal>);
      ne_local.compute(*local_normals);

      normals->points.push_back(local_normals->points[0]);

      std::cout << "local_normals->points[0] : " << local_normals->points[0] << std::endl;
    }
    else
    {
      normals->points.push_back(pcl::Normal());
    }
  }
}

int main(int argc, char **argv)
{
  PointCloudT::Ptr cloud_(new PointCloudT);
  if (pcl::io::loadPCDFile< PointT >(argv[1], *cloud_) == -1)
  {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "cloud : " << cloud_->size() << " points to pcd." << std::endl;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  // Fill cloud with data...
  estimateNormalsWithAdaptiveRadius(cloud_, cloud_normals);
  std::cout << "cloud : " << cloud_->size() << " points to pcd." << std::endl;
  std::cout << "normals : " << cloud_normals->points.size() << " points to pcd." << std::endl;
  
  // return 0;
    
    PointCloudTPtr cloud_normal_{new PointCloudT()}; 
    PointCloudTPtr cloud_outlier_{new PointCloudT()};

  for (size_t i = 0; i < cloud_normals->size(); i++)
  {
    PointT pti = cloud_->points[i];
    pcl::Normal pt = cloud_normals->points[i];

    Eigen::Vector3f this_pt_(pti.x, pti.y, pti.z);
    Eigen::Vector3f this_pt_normal(pt.normal_x, pt.normal_y, pt.normal_z);
    auto r1 = (this_pt_.normalized()).dot(this_pt_normal.normalized());
    r1 = std::fabs(r1);
    // cout << "neiji 1 : " << r1 << endl;

    if ( r1 < 0.2)  // 0.2 is good
    {
      cloud_outlier_->points.emplace_back(pti);
    }
    else
    {
      cloud_normal_->points.emplace_back(pti);
    }
  }

  std::cout << "inflation cloud points size : " << cloud_outlier_->size() << std::endl;
  std::cout << "cloud_normal_ cloud points size : " << cloud_normal_->size() << std::endl;
  cloud_outlier_->width = cloud_outlier_->size();
  cloud_outlier_->height = 1;
  cloud_normal_->width = cloud_normal_->size();
  cloud_normal_->height = 1;
  // Save output
  pcl::io::savePCDFile("/opt/csg/slam/navs/cloud_outlier_.pcd", *cloud_outlier_);
  pcl::io::savePCDFile("/opt/csg/slam/navs/cloud_normal_.pcd", *cloud_normal_);

  std::cout << "Estimated normals with adaptive radius." << std::endl;
  return 0;
}
