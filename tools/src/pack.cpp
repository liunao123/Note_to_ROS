//   pack.cpp
#include "pack.h"

pack::pack(const std::string pcd_file)
{
    // Load bun0.pcd -- should be available with the PCL archive in test
    std::cout << "pcd_file : " << pcd_file << std::endl;
    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud_) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }
    std::cout << "cloud points size : " << cloud_->size() << std::endl;
}

void pack::setInputCloud(const PointCloudTPtr& cloud)
{
    std::cout << "cloud points size : " << cloud->size() << std::endl;
    // 浅copy 同一块内存
    cloud_ = cloud;
    // pcl::copyPointCloud(*cloud, *cloud_); // 也是浅copy 同一块内存
    std::cout << "cloud points size : " << cloud_->size() << std::endl;
}

void pack::getNormalCloud(PointCloudTPtr& cloud)
{
    // 浅copy 同一块内存
    *cloud = *cloud_normal_;
}

void pack::getOutlierCloud(PointCloudTPtr& cloud)
{
    // 浅copy 同一块内存
    *cloud = *cloud_outlier_;
}

void pack::setSearchRadius(const double & value)
{
    SearchRadius_ = value;
}

void pack::setOrthogonalityTHRESHOLD(const double & value)
{
    OrthogonalityTHRESHOLD_ = value;
}

int pack::run()
{
    std::cout << SearchRadius_ << std::endl;
    std::cout << OrthogonalityTHRESHOLD_ << std::endl;
    if (cloud_ == nullptr)
    {
        std::cout << "points is null . return ." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "points is NOT null .start ...... " << std::endl;
        // Create a KD-Tree
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal> mls_points;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
        mls.setComputeNormals(true);
        // Set parameters
        mls.setInputCloud(cloud_);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        // mls.setSearchRadius(0.1);  // 0.1 is good for dense points
        mls.setSearchRadius( SearchRadius_ ); // 0.1 is good

        // Reconstruct
        mls.process(mls_points);

        std::cout << "mls_points points size : " << mls_points.size() << std::endl;

        // for (const auto pt : mls_points.points)
        for (size_t i = 0; i < mls_points.size(); i++)
        {
            pcl::PointNormal pt = mls_points[i];
            pcl::PointXYZ pti;
            // pcl::PointXYZI pti;
            pti.x = pt.x;
            pti.y = pt.y;
            pti.z = pt.z;
            // pti.intensity = cloud_i->points[i].intensity;

            Eigen::Vector3f this_pt_(pt.x, pt.y, pt.z);
            Eigen::Vector3f this_pt_normal(pt.normal_x, pt.normal_y, pt.normal_z);
            // auto r1 = this_pt_.transpose() * this_pt_normal;
            auto r1 = (this_pt_.normalized()).dot(this_pt_normal.normalized());
            r1 = std::fabs(r1);

            if ( r1 < OrthogonalityTHRESHOLD_ ) // 0.2 is good
            {
                // cout << "neiji 1 : " << r1 << endl;
                // cout << "neiji 2 : " << r2 << endl;
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
        // pcl::io::savePCDFile("/opt/csg/slam/navs/6-cloud_outlier_.pcd", *cloud_outlier_);
        // pcl::io::savePCDFile("/opt/csg/slam/navs/6-cloud_normal_.pcd", *cloud_normal_);
    }
    return 1;
}

pack::pack(/* args */)
{
    // 定义的时候如果不初始化，要在这里初始化
    // cloud_.reset(new PointCloudT());
    // cloud_normal_.reset(new PointCloudT());
    // cloud_outlier_.reset(new PointCloudT());
}

pack::~pack()
{
}
