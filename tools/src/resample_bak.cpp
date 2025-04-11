#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense> // Include the Eigen library for matrix operations

using namespace std;

// typedef pcl::PointCloud<pcl::PointXYZ> pcxyz_type;
// typedef pcl::PointCloud<pcl::PointXYZ> _type;

typedef pcl::PointCloud< pcl::PointXYZ> pcxyz_type;
// typedef pcl::PointCloud< pcl::PointXYZI > PointCloudT;

int main()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/opt/csg/slam/navs/6_1cm.pcd", *cloud) == -1)
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/opt/csg/slam/navs/0.pcd", *cloud) == -1)
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/liunao/Kalibr/v1_20241118/pcd_png/0.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }
    cout << "cloud points size : " << cloud->size() << endl;

    pcl::VoxelGrid< pcl::PointXYZ > downSizeFilterTempMap;
    double voxel_size = 0.01 ;
    std::cout << "voxel_size: " << voxel_size << std::endl;
    downSizeFilterTempMap.setLeafSize(voxel_size , voxel_size , voxel_size  );
    downSizeFilterTempMap.setInputCloud(cloud);
    downSizeFilterTempMap.filter(*cloud);
    cout << "cloud points size : " << cloud->size() << endl;

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.1);  // 0.1 is good for dense points
    mls.setSearchRadius(0.15);  // 0.1 is good 

    // Reconstruct
    mls.process(mls_points);

    cout << "cloud points size : " << mls_points.size() << endl;

    pcxyz_type::Ptr  cloud_normal(new pcxyz_type);
    pcxyz_type::Ptr cloud_inflat(new pcxyz_type);

    for (const auto pt : mls_points.points)
    {
        pcl::PointXYZ p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;

        Eigen::Vector3f this_pt_(pt.x, pt.y, pt.z);
        Eigen::Vector3f this_pt_normal(pt.normal_x, pt.normal_y, pt.normal_z);
        // auto r1 = this_pt_.transpose() * this_pt_normal;
        auto r1 = ( this_pt_.normalized() ) .dot( this_pt_normal.normalized() );
         r1 = std::fabs( r1  ) ;

        if ( r1 < 0.1 )  // 0.2 is good 
        {
            // cout << "neiji 1 : " << r1 << endl;
            // cout << "neiji 2 : " << r2 << endl;
            cloud_inflat->points.emplace_back( p );
        }
        else
        {
             cloud_normal->points.emplace_back( p );
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