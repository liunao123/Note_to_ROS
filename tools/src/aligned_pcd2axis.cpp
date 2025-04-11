#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr estimateBorders(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_plan) 
{ 
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud_plan);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setSearchMethod(kdtree);
    ne.setKSearch(10);
    // ne.setRadiusSearch( 1 ); //设置法线估计的半径
    ne.compute(*normal);

    /*pcl计算边界*/
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);     // 声明一个boundary类指针，作为返回值
    boundaries->resize(cloud_plan->size());                                                      // 初始化大小
    pcl::BoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary> boundary_estimation; // 声明一个BoundaryEstimation类
    boundary_estimation.setInputCloud(cloud_plan);                                               // 设置输入点云
    boundary_estimation.setInputNormals(normal);                                            // 设置输入法线
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZI>);
    boundary_estimation.setSearchMethod(kdtree_ptr);   // 设置搜寻k近邻的方式
    boundary_estimation.setKSearch(10);                // 设置k近邻数量
    boundary_estimation.setAngleThreshold( M_PI * 0.9 ); // 设置角度阈值，大于阈值为边界
    boundary_estimation.compute(*boundaries);          // 计算点云边界，结果保存在boundaries中
    std::cout << "boundaries->points: "   << boundaries->points.size() << std::endl;
    std::cout << "boundaries->size: "   << boundaries->size() << std::endl;

	// pcl::io::savePCDFile("/opt/csg/slam/navs/cloud_plan.pcd", *cloud_plan);

    pcl::PointCloud<pcl::PointXYZI>::Ptr line_points(new pcl::PointCloud<pcl::PointXYZI>);

    // std::cout << "line_points->size: "   << line_points->points.size() << std::endl;
    /*可视化*/
    // pcl::PointCloud<pcl::pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::pcl::PointXYZRGB>);
    // cloud_visual->resize(cloud_plan->size());
    for (size_t i = 0; i < cloud_plan->size(); i++)
    {
        // cloud_visual->points[i].x = cloud_plan->points[i].x;
        // cloud_visual->points[i].y = cloud_plan->points[i].y;
        // cloud_visual->points[i].z = cloud_plan->points[i].z;
        if ( boundaries->points[i].boundary_point > 0 )
        {
            line_points->points.push_back( cloud_plan->points[i] );
            // std::cout << " 192 line_points->size: "   << line_points->points.size() << std::endl;
            // cloud_visual->points[i].r = 255;
            // cloud_visual->points[i].g = 0;
            // cloud_visual->points[i].b = 0;
        }
        else
        {
            // cloud_visual->points[i].r = 255;
            // cloud_visual->points[i].g = 255;
            // cloud_visual->points[i].b = 255;
        }
    }
    line_points->width = line_points->size();
    line_points->height = 1;
    std::cout << "line_points->size: "   << line_points->points.size() << std::endl;
	// pcl::io::savePCDFile("/opt/csg/slam/navs/line_points.pcd", *line_points);
    return line_points;
}

const double get_line_coefficients(pcl::PointCloud<pcl::PointXYZI>::Ptr &line_points)
{
    //  拟合一条直线到点云
    pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(line_points));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model);
    ransac.setDistanceThreshold(0.025);
    ransac.computeModel();
    
    // 获取拟合出的直线参数
    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);
    
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr final(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*line_points, inliers, *final);
	pcl::io::savePCDFile("/opt/csg/slam/navs/line_points.pcd", *final);

    // 输出直线参数
    std::cout << "line model_coefficients: " << model_coefficients << std::endl
              << std::endl;
    std::cout << "inliers : " << inliers.size()   << std::endl;
    std::cout << "inliers/total: " << 1.0 * inliers.size() / line_points->size()   << std::endl;
    
    double dx = model_coefficients[3]; // x 方向分量
    double dy = model_coefficients[4]; // y 方向分量

    // 计算直线的斜率
    double slope = - std::atan2( dy, dx ) ;
    std::cout << "slope angle_deg: " << slope * 180.0 / M_PI << " " << std::endl;
    return slope;
}


int main(int argc, char** argv) 
{
	std::string filename = "/opt/csg/slam/navs/test.pcd";
	if (argc == 2)
	{
		filename = argv[1];
	}
    std::cout << "filename: " << filename << std::endl;

    /*输入点云和法线*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud_o) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    std::cout << "Saved cloud_o size: " << cloud_o->size() << std::endl;

    // 深copy
    // *cloud_filtered = *cloud_o;
    // std::cout << "Saved cloud_filtered size: " << cloud_filtered->size() << std::endl;
    // // 注意pcd文件中的VIEWPOINT
    // std::cout << "sensor_orientation_: " << cloud_filtered->sensor_orientation_.coeffs() << std::endl;
    // std::cout << "sensor_origin_: " << cloud_filtered->sensor_origin_ << std::endl;
    // std::cout << "sensor_origin_: " << &cloud_filtered << std::endl;
    // std::cout << "132: " << cloud_o << std::endl;
    // std::cout << "133: " << cloud_filtered << std::endl;
    // cloud_filtered->clear();
    // std::cout << "Saved cloud_o size: " << cloud_o->size() << std::endl;
    // std::cout << "Saved cloud_filtered size: " << cloud_filtered->size() << std::endl;

    // pcl::io::savePCDFile("/opt/csg/slam/navs/111-1.pcd", *cloud_o );

    std::cout << "Saved voxel before: " << cloud_o->size() << " data points to test_pcd.pcd." << std::endl;
	pcl::VoxelGrid< pcl::PointXYZI > downSizeFilterTempMap;
    const double reslutiuon = 0.10;
    downSizeFilterTempMap.setLeafSize(reslutiuon, reslutiuon, reslutiuon);
    downSizeFilterTempMap.setInputCloud(cloud_o);
    downSizeFilterTempMap.filter(*cloud_filtered);
    std::cout << "Saved voxel after: " << cloud_filtered->size() << " data points to test_pcd.pcd." << std::endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plan(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0  , 10.5 );
    pass.filter(*cloud_plan);
    std::cout << "cloud: " <<   cloud_plan->size() << std::endl;
    // *cloud_plan = *cloud_filtered;


    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D( *cloud_plan, minPt, maxPt);

    auto line_points = estimateBorders(cloud_plan);
 
    double slope = get_line_coefficients(line_points);
  
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf( slope , Eigen::Vector3f::UnitZ())); // 绕Z轴旋转 
    transform(0, 3) = (minPt.x + maxPt.x) / 2;
    transform(1, 3) = (minPt.y + maxPt.y) / 2;

    std::cout << "transform: " << std::endl <<  transform.matrix() << std::endl;
    // 对点云进行变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_o, *transformedCloud, transform);

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // 可视化原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red_color(line_points, 255, 0, 0);
    viewer.addPointCloud(line_points, red_color, "line_points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> blue_color(cloud_plan,  0, 0,255);
    viewer.addPointCloud(cloud_plan, blue_color, "cloud_o");

    // 添加第二个点云，设置为green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> green_color(transformedCloud, 0, 255, 0);
    viewer.addPointCloud(transformedCloud, green_color, "transformedCloud");

    transformedCloud->width = transformedCloud->size();
    transformedCloud->height = 1;
    pcl::io::savePCDFile("/opt/csg/slam/navs/trans.pcd", *transformedCloud);
    std::cout << "transformedCloud save done : " <<   transformedCloud->size() << std::endl;

    viewer.spin();
    return 0;
}
