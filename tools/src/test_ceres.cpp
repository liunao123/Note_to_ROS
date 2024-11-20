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
    residual[0] = T ( p2_.x_ -   four[0] -  four[3] *  ( p1_.x_ * ceres::cos(four[2]) - p1_.y_ * ceres::sin(four[2]) )    );
    residual[1] = T ( p2_.y_ -   four[1] -  four[3] *  ( p1_.x_ * ceres::sin(four[2]) + p1_.y_ * ceres::cos(four[2]) )    );
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
 
    double four_init[4] = {0 , 0, 3.1 , 1};

    Problem problem; // 求解对象（Problem应该是顶层的抽象类）
    problem.AddParameterBlock(four_init, 4); // 添加向量 x 作为参数

    std::ifstream file("../p2.txt");
    std::string line;
    while (std::getline(file, line))
    {
      std::istringstream iss(line);
      double  x1,  y1, x2, y2;

      if (!(iss >>  x1 >>  y1 >> x2 >> y2))
      {
        // 解析失败
        std::cerr << "解析行失败: " << line << std::endl;
        // continue;
      }
      std::cout << std::fixed << std::setprecision(10) ;
      std::cout << "数据1: " <<  x1 << " 数据2: " <<  y1 << " 数据3: " << x2 << " 数据4: " << y2 << std::endl;
      point p1(x1,  y1);
      point p2(x2,  y2);

      vp1.push_back(p1);
      vp2.push_back(p2);

      // point p1(y1,  x1);
      // point p2(y2,  x2);
      // 查找最近邻点
      // 小于某个阈值认为查找到了
      // 把这两个点加进去
      std::cout << "x " << p2.x_ << "   y " << p2.y_  << "\n";

      CostFunction *cost_function =                                                             // 《14讲》把这个省略，直接写到AddResidualBlock中，其实一样的
          new AutoDiffCostFunction<ExponentialResidual, 2, 4>(new ExponentialResidual(p1, p2)); // 前面定义的仿函数的构造函数初始化，将数据传入
      problem.AddResidualBlock(cost_function, nullptr, four_init);

      // 打印读取的数据
    }
    file.close();

    problem.SetParameterLowerBound(four_init, 2, 0.0 ); // 半径不能为负值
    problem.SetParameterUpperBound(four_init, 2, 2*M_PI); // 半径不能为负值

    problem.SetParameterLowerBound(four_init, 3, 0.0); 

    
    // The options structure, which controls how the solver operates
    Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_SCHUR ;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Run solver
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << std::fixed << std::setprecision(5) ;
    std::cout << "result : " <<  "  " <<  four_init[0]    << "   " << four_init[1] << "   " <<    NormalizeAngle<double> ( four_init[2] )  << "   " << four_init[3]  << "\n";

    for ( size_t i = 0; i < vp1.size(); i++ )
    {
      double dx =   vp2[i].x_ -   four_init[0] -  four_init[3] *  ( vp1[i].x_ * ceres::cos(four_init[2]) - vp1[i].y_ * ceres::sin(four_init[2]) ) ;
      double dy =   vp2[i].y_ -   four_init[1] -  four_init[3] *  ( vp1[i].x_ * ceres::sin(four_init[2]) + vp1[i].y_ * ceres::cos(four_init[2]) ) ;
      std::cout << "residual : "  <<  dx    << "   " << dy   << "\n";
    }

}