#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/octree/octree.h> 

struct initial_parameters
{
    /* data */
    std::string camera_topic;
    std::string pts_topic;
    std::string pose_topic;
    cv::Mat camtocam_mat;
    cv::Mat cameraIn;
    cv::Mat RT;
    double cam_d0, cam_d1, cam_d2, cam_d3;
    int cam_width, cam_height;
} i_params;

 
void initParams(ros::NodeHandle &nh)
{
    double_t camtocam[12] = {0.0};
    double_t cameraIn[16] = {0.0};
    double_t RT[16] = {0.0};

    // parameter from ros
    nh.param<std::string>("camera_topic", i_params.camera_topic, "/image");
    nh.param<std::string>("pts_topic", i_params.pts_topic, "/undistort_laser");
    nh.param<std::string>("pose_topic", i_params.pose_topic, "/lidar_pose");

    // std::cout << "camera_topic: " << i_params.camera_topic << std::endl;
    // std::cout << "pts_topic: " << i_params.pts_topic << std::endl;
    // std::cout << "pose_topic: " << i_params.pose_topic << std::endl;

    double cam_fx = 0, cam_fy = 0, cam_cx = 0, cam_cy = 0;
    nh.param<double>("cam_fx", cam_fx, 453.483063);
    nh.param<double>("cam_fy", cam_fy, 453.254913);
    nh.param<double>("cam_cx", cam_cx, 318.908851);
    nh.param<double>("cam_cy", cam_cy, 234.238189);
    cameraIn[0] = cam_fx;
    cameraIn[5] = cam_fy;
    cameraIn[2] = cam_cx;
    cameraIn[6] = cam_cy;
    cameraIn[10] = 1.0;
    cameraIn[15] = 1.0;
    cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn); // cameratocamera params
    std::cout << __FILE__ << ":" << __LINE__ << std::endl << i_params.cameraIn << std::endl;

    // 定义相机的畸变参数
    nh.param<double>("cam_d0", i_params.cam_d0, -0.0971610);
    nh.param<double>("cam_d1", i_params.cam_d1, 0.1481190);
    nh.param<double>("cam_d2", i_params.cam_d2, -0.0017345);
    nh.param<double>("cam_d3", i_params.cam_d3, 0.0006040);
    nh.param<int>("cam_width", i_params.cam_width, 512);
    nh.param<int>("cam_height", i_params.cam_height, 612);
    std::cout << __FILE__ << ":" << __LINE__ << " distortionCoeffs: " << std::endl;
    std::cout << i_params.cam_d0 << std::endl;
    std::cout << i_params.cam_d1 << std::endl;
    std::cout << i_params.cam_d2 << std::endl;
    std::cout << i_params.cam_d3 << std::endl;
    std::cout << "cam_width : " << i_params.cam_width << std::endl;
    std::cout << "cam_height : " << i_params.cam_height << std::endl;

    std::vector<double> cameraextrinT(3, 0.0);
    std::vector<double> cameraextrinR(9, 0.0);
    std::cout << " try to get camera/Pcl and camera/Rcl param:" << std::endl;

    nh.param<std::vector<double>>("camera/Pcl", cameraextrinT, std::vector<double>());
    nh.param<std::vector<double>>("camera/Rcl", cameraextrinR, std::vector<double>());
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            RT[i * 4 + j] = cameraextrinR[i * 3 + j];
        }
    }
    RT[3] = cameraextrinT[0];
    RT[7] = cameraextrinT[1];
    RT[11] = cameraextrinT[2];
    RT[15] = 1;
    cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT); // lidar to camera params
    std::cout << __FILE__ << ":" << __LINE__ << std::endl
              << i_params.RT << std::endl;
}

const std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr , cv::Mat >  generate_lidar_intensity_img( const std::string pcd_filename )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file\n");
    }
    std::cout << __FILE__ <<":" << __LINE__ << " points size:  " << cloud->size() << std::endl;
    // pcl::PassThrough< pcl::PointXYZI > pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("intensity");
    // pass.setFilterLimits (0.0, 50);
    // //pass.setNegative (true);
    // pass.filter (*cloud);
    // std::cout << __FILE__ <<":" << __LINE__ << " points size:  " << cloud->size() << std::endl;


    // static pcl::VoxelGrid< pcl::PointXYZI > downSizeFilterTempMap;
    // const float resolution = 0.03f; // 指定分辨率
    // 太低的分辨率会使得强度图不连续，导致会有很多小块被认为是边缘
    // downSizeFilterTempMap.setLeafSize( resolution, resolution , resolution );
    // downSizeFilterTempMap.setInputCloud(cloud);
    // downSizeFilterTempMap.filter(*cloud);
    // std::cout << __FILE__ <<":" << __LINE__ << " points size:  " << cloud->size() << std::endl;

    cv::Mat intensity_image(i_params.cam_width, i_params.cam_height, CV_64FC1, cv::Scalar::all(0));
    cv::Mat index_image(i_params.cam_width, i_params.cam_height, CV_32SC1, cv::Scalar::all(-1));

    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(4, 1, cv::DataType<double>::type);

    for (int it = 0; it < cloud->points.size(); it++)
    {
        X.at<double>(0, 0) = cloud->points[it].x;
        X.at<double>(1, 0) = cloud->points[it].y;
        X.at<double>(2, 0) = cloud->points[it].z;
        X.at<double>(3, 0) = 1;

        Y = i_params.cameraIn * i_params.RT * X; // tranform the point to the camera coordinate

        cv::Point pt;
        pt.x = std::round (Y.at<double>(0, 0) / Y.at<double>(0, 2) );
        pt.y = std::round (Y.at<double>(0, 1) / Y.at<double>(0, 2) );

        // 移除边缘的点
        const int remove_pixel_thres = 2;
        if (pt.y < remove_pixel_thres || pt.y > (i_params.cam_height - remove_pixel_thres) ||
            pt.x < remove_pixel_thres || pt.x > (i_params.cam_width - remove_pixel_thres))
        {
            continue;
        }

        // 生成对应像素的强度
        // 记录对应这个像素点的雷达点索引
        intensity_image.at< double >(pt.y, pt.x) =  cloud->points[it].intensity;
        index_image.at< int >( pt.y, pt.x ) = it ;
    }

    cv::Mat indices_8uc4(index_image.rows, index_image.cols, CV_8UC4, reinterpret_cast<cv::Vec4b*>(index_image.data));
    cv::imwrite("/opt/csg/slam/navs/lidar_indices.png", indices_8uc4);

    cv::imwrite("/opt/csg/slam/navs/1.png", intensity_image);
    cv::Mat intensity_image_equa;

    cv::normalize(intensity_image, intensity_image_equa, 0, 255, cv::NORM_MINMAX);
    cv::imwrite("/opt/csg/slam/navs/1_equa.png", intensity_image_equa);

    intensity_image_equa.clone().convertTo(intensity_image_equa, CV_8UC1, 1.0 );
    cv::imwrite("/opt/csg/slam/navs/1_CV_8UC1.png", intensity_image_equa);

    cv::equalizeHist(intensity_image_equa, intensity_image_equa);
    cv::imwrite("/opt/csg/slam/navs/1_equa.png", intensity_image_equa);

    // intensity_image_equa = cv::imread("/home/liunao/Kalibr/v1_20241118/pcd_png/4.bag_lidar_intensities_CV_8UC1.png", cv::IMREAD_GRAYSCALE);
    // intensity_image_equa = cv::imread("/opt/csg/slam/navs/1_equa.png", cv::IMREAD_GRAYSCALE);

    // 提取强度图像的边缘
    cv::Mat edge;
    cv::Canny(intensity_image_equa, edge, 150, 200);
    cv::imwrite("/opt/csg/slam/navs/intensity_image_equa_edge.png", edge);

    // cv::Mat laplacian_edge;
    // cv::Laplacian(intensity_image_equa, laplacian_edge, CV_16S, 3); // CV_16S 是输出图像的深度，3 是卷积核的大小
    // cv::convertScaleAbs(laplacian_edge, laplacian_edge); // 转换为可显示的图像
    // cv::imwrite("/opt/csg/slam/navs/intensity_image_equa_edge_laplacian.png", laplacian_edge);

    // 根据图像的边缘来提取对应的雷达点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts_edge(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pts_edge_lap(new pcl::PointCloud<pcl::PointXYZI>());

    for (int row = 0; row < edge.rows; ++row) {
      for (int col = 0; col < edge.cols; ++col) {
        // 访问每个元素
        uchar pixelValue = edge.at<uchar>(row, col);
        // 处理 pixelValue
        if (pixelValue)
        {
          auto ind = index_image.at<int>(row, col);
          if (ind >= 0 && ind < cloud->points.size())
          {
            pts_edge->points.push_back(cloud->points[ind]);
          }
        }

        // uchar pixelValue_lap = laplacian_edge.at<uchar>(row, col);
        // // 处理 pixelValue
        // if (pixelValue_lap)
        // {
        //   auto ind = index_image.at<int>(row, col);
        //   if (ind >= 0 && ind < cloud->points.size())
        //   {
        //     pts_edge_lap->points.push_back(cloud->points[ind]);
        //   }
        // }

      }
    }
    /* // !todo  remove small point cluster    */
    pcl::RadiusOutlierRemoval< pcl::PointXYZI > outrem;
    outrem.setInputCloud(pts_edge);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius( 10 );
    // apply filter
    outrem.filter(*pts_edge);
    
    pts_edge->width = pts_edge->points.size();
    pts_edge->height = 1;
    pcl::io::savePCDFile("/opt/csg/slam/navs/lidar_edge.pcd", *pts_edge);
    std::cout << "save lidar_edge done. pts: " << pts_edge->width << std::endl;

    // pts_edge_lap->width = pts_edge_lap->points.size();
    // pts_edge_lap->height = 1;
    // pcl::io::savePCDFile("/opt/csg/slam/navs/lidar_edge_lap.pcd", *pts_edge_lap);

    return std::make_pair( pts_edge, edge );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_lidar_dege");
    
    std::string data_path = "/home/direct_l_v_calibrate_data_avia/cab_hall/pcd_png";
    int PCD_NUM = 1 ;

    ros::NodeHandle nh;
    initParams(nh);
    
    nh.param<std::string>("data_path",data_path, "/home/map/rgb_test" );
    ROS_WARN("data_path is %s . ", data_path.c_str() );

    nh.param<int>("PCD_NUM", PCD_NUM, 2);
    ROS_WARN("img and pcd num is %d  . ", PCD_NUM );

    // ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/rgb_pts", 10000);
    // image_transport::ImageTransport imageTransport(nh);
    // image_transport::Publisher image_publisher = imageTransport.advertise("/project_pc_image", 10000);

    for (int i = 6; i < PCD_NUM ; i++ )
    {
        std::string pcd_file = data_path + "/" + std::to_string(i)  + ".pcd" ;
        std::string rgb_pcd_file = data_path + "/" + std::to_string(i)  + "_edge.pcd" ;
        std::string pts_img_file = data_path + "/" + std::to_string(i)  + "_intensity_edge.png" ;
        ROS_WARN("loading %s . ", pcd_file.c_str());
        try
        {
            auto rgb_ponts_and_img = generate_lidar_intensity_img(pcd_file);
            pcl::io::savePCDFile(rgb_pcd_file, *( rgb_ponts_and_img.first ) );
            cv::imwrite(pts_img_file, rgb_ponts_and_img.second );
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

    }

    return 0;
}