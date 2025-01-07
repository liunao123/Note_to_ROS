#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense> // Include the Eigen library for matrix operations

using namespace std;

// typedef pcl::PointCloud<PointT> PointCloudT;
// typedef pcl::PointCloud<PointT> _type;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

int main()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    if (pcl::io::loadPCDFile<PointT>("/opt/csg/slam/navs/6_1cm.pcd", *cloud) == -1)
    // if (pcl::io::loadPCDFile<PointT>("/opt/csg/slam/navs/0.pcd", *cloud) == -1)
    // if (pcl::io::loadPCDFile<PointT>("/home/liunao/Kalibr/v1_20241118/pcd_png/0.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("/opt/csg/slam/navs/6_1cm.pcd", *cloud_i) == -1)
    // if (pcl::io::loadPCDFile<PointT>("/opt/csg/slam/navs/0.pcd", *cloud) == -1)
    // if (pcl::io::loadPCDFile<PointT>("/home/liunao/Kalibr/v1_20241118/pcd_png/0.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }

    // cloud->points.resize( cloud_i->size() );
    // for (size_t i = 0; i <  cloud_i->size() ; i++)
    // {
    //     // cout << "cloud_i  : " << cloud_i->points[i].x << " " << cloud_i->points[i].y << " " << cloud_i->points[i].z   << endl;
    //     // cout << "cloud  : " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z   << endl;
    //     cloud->points[i].x =  cloud_i->points[i].x;
    //     cloud->points[i].y =  cloud_i->points[i].y;
    //     cloud->points[i].z =  cloud_i->points[i].z;
    // }

    cout << "cloud points size : " << cloud->size() << endl;

    // pcl::VoxelGrid< PointT > downSizeFilterTempMap;
    // double voxel_size = 0.01 ;
    // std::cout << "voxel_size: " << voxel_size << std::endl;
    // downSizeFilterTempMap.setLeafSize(voxel_size , voxel_size , voxel_size  );
    // downSizeFilterTempMap.setInputCloud(cloud);
    // downSizeFilterTempMap.filter(*cloud);
    // cout << "cloud points size : " << cloud->size() << endl;

    // Create a KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.1);  // 0.1 is good for dense points
    mls.setSearchRadius(0.1);  // 0.1 is good 

    // Reconstruct
    mls.process(mls_points);

    cout << "mls_points points size : " << mls_points.size() << endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud_normal(new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inflat(new pcl::PointCloud<pcl::PointXYZI> );

    // for (const auto pt : mls_points.points)
    for (size_t i = 0; i <  mls_points.size(); i++)
    {
        pcl::PointNormal pt = mls_points[ i ];
        pcl::PointXYZI pti;
        pti.x = pt.x;
        pti.y = pt.y;
        pti.z = pt.z;
        pti.intensity = cloud_i->points[i].intensity;

        if ( i % 10000 ==  0)
        {
        cout << "cloud  : " << cloud_i->points[i].x << " " << cloud_i->points[i].y  << " "   << cloud_i->points[i].z <<  endl;
        cout << "mls_points  : " << mls_points.points[i].x << " " << mls_points.points[i].y  << " "   << mls_points.points[i].z <<  endl;
        }

        Eigen::Vector3f this_pt_(pt.x, pt.y, pt.z);
        Eigen::Vector3f this_pt_normal(pt.normal_x, pt.normal_y, pt.normal_z);
        // auto r1 = this_pt_.transpose() * this_pt_normal;
        auto r1 = ( this_pt_.normalized() ) .dot( this_pt_normal.normalized() );
        r1 = std::fabs(r1);

        if (r1 < 0.15) // 0.2 is good
        {
            // cout << "neiji 1 : " << r1 << endl;
            // cout << "neiji 2 : " << r2 << endl;
            cloud_inflat->points.emplace_back(pti);
        }
        else
        {
            cloud_normal->points.emplace_back(pti);
        }
    }

    cout << "inflation cloud points size : " << cloud_inflat->size() << endl;
    cout << "cloud_normal cloud points size : " << cloud_normal->size() << endl;
    cloud_inflat->width = cloud_inflat->size();
    cloud_inflat->height = 1;
    cloud_normal->width = cloud_normal->size();
    cloud_normal->height = 1;
    // Save output
    pcl::io::savePCDFile("/opt/csg/slam/navs/6-cloud_inflat.pcd", *cloud_inflat);
    pcl::io::savePCDFile("/opt/csg/slam/navs/6-cloud_normal.pcd", *cloud_normal);
}