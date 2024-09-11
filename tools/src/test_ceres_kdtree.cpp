#include "ceres/ceres.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <vector>

#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
struct point {
  point(double x, double y)  // 重写了结构体的构造函数，用于外部传递数据
      : x_(x), y_(y) {}
  double x_;
  double y_;
};

struct ExponentialResidual {
  ExponentialResidual(point p1, point p2)  // 重写了结构体的构造函数，用于外部传递数据
      : p1_(p1), p2_(p2) {}

  template <typename T>
  bool operator()(const T* const four,  T* residual) const {
    residual[0] = T ( p2_.x_ -   four[0] -   ( p1_.x_ * ceres::cos(four[2]) - p1_.y_ * ceres::sin(four[2]) )    );
    residual[1] = T ( p2_.y_ -   four[1] -   ( p1_.x_ * ceres::sin(four[2]) + p1_.y_ * ceres::cos(four[2]) )    );
      // std::cout << "four[0] " << four[0]   << "\n";
      // std::cout << "four[1] " << four[1]   << "\n";
    // std::cout << "operator : " <<  "  " <<  four[0]   << "   " << four[1] << "   " << four[2] << "   " << four[3]  << "\n";
    return true;
  }

 private:
  // Observations for a sample.
  // const double x_;
  // const double y_;
  point p1_;
  point p2_;
};

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

int main() 
{
    const int kNumObservations = 200;

    std::vector< point >  vp1;
    std::vector< point >  vp2;
 
    double robot_pose[4] = {17.0 , 27.0, 0.0};

    Problem problem; // 求解对象（Problem应该是顶层的抽象类）
    problem.AddParameterBlock(robot_pose, 3); // 添加向量 x 作为参数

    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr one_scan(new pcl::PointCloud<pcl::PointXYZ>);

    std::string map_file = "/opt/csg/slam/navs/temp/data/global_map.pcd";
    std::string scan_file = "/opt/csg/slam/navs/temp/data/one_scan.pcd";

    std::cout << "117 map: " << map_file << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, *map) == -1)
    {
      printf("Couldn't read file\n");
      return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_file, *one_scan) == -1)
    {
      printf("Couldn't read file\n");
      return -1;
    }

    std::cout << "Saved voxel before: " << map->size() << " map points  " << std::endl;
	  pcl::VoxelGrid< pcl::PointXYZ > downSizeFilterTempMap;
    const double reslutiuon = 0.10;
    downSizeFilterTempMap.setLeafSize(reslutiuon, reslutiuon, reslutiuon);
    downSizeFilterTempMap.setInputCloud(map);
    downSizeFilterTempMap.filter(*map);
    std::cout << "Saved voxel after: " << map->size() << " map points  " << std::endl;
    // reader.read("*.ply", *normal);

    // 创建 KD 树对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(map);
 
    int K = 1;  // 搜索最近的一个点
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);


    for (size_t i = 0; i < 5; i++)
    {
    
    int cnts = 0;

    for (const auto pt : one_scan->points  )
    {
      // 搜索最近邻点
      pcl::PointXYZ search_point;
      // search_point = pt;

      // search_point.x =  robot_pose[0] +  ( pt.x * ceres::cos(robot_pose[2]) - pt.y * ceres::sin(robot_pose[2]) );
      // search_point.y =  robot_pose[1] +  ( pt.x * ceres::sin(robot_pose[2]) + pt.y * ceres::cos(robot_pose[2]) );

      if ( std::isnan( pt.x ) || std::isnan( pt.y ) || std::isnan( pt.z ))
      {
        continue;
      }
      
      // // 先平移 后旋转
      double temp_x = pt.x + robot_pose[0];
      double temp_y = pt.y + robot_pose[1];
      search_point.x =  temp_x * ceres::cos(robot_pose[2]) - temp_y * ceres::sin(robot_pose[2]) ;
      search_point.y =  temp_x * ceres::sin(robot_pose[2]) + temp_y * ceres::cos(robot_pose[2]) ;

      if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        size_t i = 0;
        for (; i < pointIdxNKNSearch.size(); ++i)
        {
          // std::cout << "Nearest point " << i << ": " << map->points[pointIdxNKNSearch[i]].x
          //           << " " << map->points[pointIdxNKNSearch[i]].y
          //           << " " << map->points[pointIdxNKNSearch[i]].z
          //           << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;

          if (pointNKNSquaredDistance[i] > 0.04)
          {
            // break;
          }
          else
          {
            cnts++;
            point p1(map->points[pointIdxNKNSearch[i]].x, map->points[pointIdxNKNSearch[i]].y);
            point p2(search_point.x, search_point.y);

            CostFunction *cost_function =                                                             // 《14讲》把这个省略，直接写到AddResidualBlock中，其实一样的
                new AutoDiffCostFunction<ExponentialResidual, 2, 3>(new ExponentialResidual(p1, p2)); // 前面定义的仿函数的构造函数初始化，将数据传入
            problem.AddResidualBlock(cost_function, nullptr, robot_pose);
          }
        }
      }
    }


    // return 9;

    // The options structure, which controls how the solver operates
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR ;
    // options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Run solver
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << std::fixed << std::setprecision(5) ;
    std::cout << "result : " <<  "  " <<  robot_pose[0]    << "   " << robot_pose[1] << "   " <<    NormalizeAngle<double> ( robot_pose[2] )  << "\n";
    std::cout << "size : "  << cnts << "  percernt: " << 1.0 * cnts / one_scan->points.size()   << "\n";

    }
    // for ( size_t i = 0; i < vp1.size(); i++ )
    // {
    //   double dx =   vp2[i].x_ -   robot_pose[0] -   ( vp1[i].x_ * ceres::cos(robot_pose[2]) - vp1[i].y_ * ceres::sin(robot_pose[2]) ) ;
    //   double dy =   vp2[i].y_ -   robot_pose[1] -   ( vp1[i].x_ * ceres::sin(robot_pose[2]) + vp1[i].y_ * ceres::cos(robot_pose[2]) ) ;
    //   std::cout << "residual : "  <<  dx    << "   " << dy   << "\n";
    // }

}