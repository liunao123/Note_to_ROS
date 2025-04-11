//   pack.cpp
#include "pack.h"

pack::pack(const std::string pcd_file)
{
    // Load bun0.pcd -- should be available with the PCL archive in test
    std::cout << "pcd_file : " << pcd_file << std::endl;
    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud_) == -1)
    {
      std::cout << "Could not read file\n" << std::endl;
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
    std::cout << " 194 SearchRadius_: " << SearchRadius_ << std::endl;
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
        /*

        // Create a KD-Tree
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal> cloud_normals;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares< pcl::PointXYZ , pcl::PointNormal> mls;
        mls.setComputeNormals(true);
        // Set parameters
        mls.setInputCloud(cloud_);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        // mls.setSearchRadius(0.1);  // 0.1 is good for dense points
        mls.setSearchRadius( SearchRadius_ ); // 0.1 is good

        // Reconstruct
        mls.process(cloud_normals);
        // std::cout << "pts: " << cloud_->size() << std::endl;
        */
        // for (size_t i = 0; i < cloud_normals.size(); i++)
        // {
        //     pcl::PointNormal pt = cloud_normals.points[i];
        //     std::cout << "xyz: " << cloud_->points[i].x << " " << cloud_->points[i].y << " " << cloud_->points[i].z   << std::endl;
        //     std::cout << "xyz: " << pt.x << " " << pt.y << " " << pt.z   << std::endl;
        //     std::cout << "xyz normal: " << pt.normal_x << " " << pt.normal_y << " " << pt.normal_z   << std::endl;
        // }

        // 估计法线
        /*        */
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud_);
        // 创建一个空的kdtree对象，并把它传递给法线估计对象
        // 基于给出的输入数据集，kdtree将被建立
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);
        // 输出数据集
        pcl::PointCloud<pcl::Normal>  cloud_normals;
        // 使用半径在查询点周围3厘米范围内的所有邻元素
        // ne.setRadiusSearch(0.1);
        ne.setRadiusSearch( SearchRadius_ );
        // ne.setKSearch(20);
        
        // 计算特征值
        ne.compute(cloud_normals);


        std::cout << "cloud points size : " << cloud_->size() << std::endl;
        std::cout << "cloud_normals points size : " << cloud_normals.size() << std::endl;

        // viewer for debug
        // 创建可视化工具对象
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);
        // viewer->addPointCloud<pcl::PointXYZ>(cloud_ , "sample cloud");
        
        // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_, cloud_normals.makeShared(), 10, 0.05, "normals"); // 法线长度为0.05m，箭头大小为10px

        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        // viewer->addCoordinateSystem(1.0);
        // viewer->initCameraParameters();

        // // 进入可视化循环
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(100);
        //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        // }
        // viewer for debug

        // 计算每个点的法向量与射线方向的夹角
        for (size_t i = 0; i < cloud_normals.size(); i++)
        {
            PointT pti = cloud_->points[i];
            pcl::Normal pt = cloud_normals.points[i];

            // PointT pti;
            // pti.x = cloud_normals.points[i].x;
            // pti.y = cloud_normals.points[i].y;
            // pti.z = cloud_normals.points[i].z;
            // pcl::Normal pt;
            // pt.normal_x = cloud_normals.points[i].normal_x;
            // pt.normal_y = cloud_normals.points[i].normal_y;
            // pt.normal_z = cloud_normals.points[i].normal_z;

            Eigen::Vector3f this_pt_(pti.x, pti.y, pti.z);
            Eigen::Vector3f this_pt_normal(pt.normal_x, pt.normal_y, pt.normal_z);
            // auto r1 = (this_pt_.normalized()).dot(this_pt_normal.normalized());
            // r1 = std::fabs(r1);

            double cosValNew = std::fabs( this_pt_.dot(this_pt_normal) / (this_pt_.norm() * this_pt_normal.norm()) ); // 角度cos值
            double angleNew = std::acos(cosValNew) * 180 / M_PI;          // 角度
            // std::cout << "angleNew: " << angleNew << "  . cosValNew : " << cosValNew << "  . r1 : " << r1 << std::endl;

            // if ( cosValNew < 0.2 ) // 0.2 is good
            // if ( r1 < OrthogonalityTHRESHOLD_ ) // 0.2 is good
            // if ( angleNew < 95.0 && angleNew > 85.0 ) // 0.2 is good
            if ( angleNew < 100.0 && angleNew > 80.0 ) // cos(80) = 0.17 is good
            {
                cloud_outlier_->points.emplace_back(pti);
            }
            else
            {
                cloud_normal_->points.emplace_back(pti);
            }
        }

        // std::cout << "PassThrough intensity.  points size : " << cloud_normal_->size() << std::endl;
        // pcl::PassThrough<PointT> pass;
        // pass.setInputCloud(cloud_normal_);
        // pass.setFilterFieldName("intensity");
        // // pass.setFilterLimits (0.2, 0.7);
        // pass.setFilterLimits(0, 5);
        // pass.setNegative(true);
        // pass.filter(*cloud_normal_);
        // std::cout << "PassThrough intensity.  points size : " << cloud_normal_->size() << std::endl;

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
