#include "ceres/ceres.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <vector>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
struct point
{
  point(double x, double y) // 重写了结构体的构造函数，用于外部传递数据
      : x_(x), y_(y)
  {
  }
  double x_;
  double y_;
};

struct ExponentialResidual
{
  ExponentialResidual(point p1) // 重写了结构体的构造函数，用于外部传递数据
      : pt(p1)
  {
  }

  template <typename T>
  bool operator()(const T *const four, T *residual) const
  {
    // residual[0] =  (pt.x_ -  four[0] )*(pt.x_ -  four[0] ) +  (pt.y_ -  four[1] )*(pt.y_ -  four[1] )  - four[2] * four[2] ; // ok
    residual[0] = four[2]*four[2] - (pt.x_ - four[0])*(pt.x_ - four[0]) - (pt.y_ -  four[1] )*(pt.y_ -  four[1] );

    // residual[0] = T(four[2])*T(four[2]) - (pt.x_ - T(four[0]))*(pt.x_ - T(four[0])) - (pt.y_ - T(four[1]))*(pt.y_ - T(four[1])) ;
    // residual[0] =  (T(pt.x_) - four[0])*(T(pt.x_) - four[0]) + (T(pt.y_) - four[1])*(T(pt.y_) - four[1]) -  four[2]*four[2] ;
    // residual[0] = -  (pt.x_ - four[0])  / ceres::sqrt( (four[2]*four[2]) - (pt.x_ - four[0])*(pt.x_ - four[0]) )  ;

    // residual[0] = (T(x_) - *center_x) * (T(x_) - *center_x) + (T(y_) - *center_y) * (T(y_) - *center_y) - *radius * *radius;
    // std::cout << "operator : " <<  "  " <<  four[0]   << "   " << four[1] << "   " << four[2] << "   " << four[3]  << "\n";
    return true;
  }

private:
  // Observations for a sample.
  // const double x_;
  // const double y_;
  point pt;
};

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
T NormalizeAngle(const T &angle_radians)
{
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}


std::vector<point> read_pose(std::string filename)
{
  std::vector<point> pose_vec;
  std::fstream file;
  file.open(filename);
  double st, tx, ty, tz, w, x, y, z;
  std::cout << "pose filename is " << filename << std::endl;
  while (!file.eof())
  {
    file >> st >> tx >> ty >> tz >> x >> y >> z >> w;
    point pt(tx, ty);
    pose_vec.emplace_back(pt);
    // printf("pt %f  ,  %f", tx, ty);
  }
  file.close();
  pose_vec.pop_back();
  std::cout << "pose size is " << pose_vec.size() << std::endl;
  return pose_vec;
}

int main()
{
  // const int kNumObservations = 200;
  double four_init[3] = {0.0, 0.0, 0.40};

  Problem problem;                         // 求解对象（Problem应该是顶层的抽象类）
  problem.AddParameterBlock(four_init, 3); // 添加向量 x 作为参数

  std::string filename = "../data/circle_path.txt";
  auto pts = read_pose(filename);

  for (const auto pt : pts)
  {
    CostFunction *cost_function =                                                             // 《14讲》把这个省略，直接写到AddResidualBlock中，其实一样的
        new AutoDiffCostFunction<ExponentialResidual, 1, 3>(new ExponentialResidual(pt));     // 前面定义的仿函数的构造函数初始化，将数据传入
    problem.AddResidualBlock(cost_function, nullptr, four_init);
  }
  // problem.SetParameterLowerBound(four_init, 2, 0.20); // 半径不能为负值

  // The options structure, which controls how the solver operates
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR ;
  // options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100; // 设置最大迭代次数为 100
  options.function_tolerance = 1e-8; // 设置函数值的收敛阈值为 1e-6

  // Run solver
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "used time : " << summary.total_time_in_seconds  << "\n";
  
  std::cout << std::fixed << std::setprecision(5);
  std::cout << "result : " << "  " << four_init[0] << "   " << four_init[1] << "   " <<  four_init[2]  << "\n";

  double circle_x  = four_init[0];
  double circle_y  = four_init[1];
  double circle_radius  = four_init[2];

  // for (double i = 0; i < 3600; i++)
  double angle = 0;
  while (  angle < 2*M_PI + 0.1)
  {
    double sample_x  =  circle_x + circle_radius * std::cos( angle );
    double sample_y  =  circle_y + circle_radius * std::sin( angle );
    // std::cout << "0 " <<  sample_x << " " << sample_y << " 0 0 0 0 1" << std::endl;
    angle = angle + 0.01;
  }
  
}