#include <iostream>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

#include <random>
#include <ctime>

#include "ceres/ceres.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <vector>

using namespace std;

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
  ExponentialResidual(point p1)  // 重写了结构体的构造函数，用于外部传递数据
      : p1_(p1) {}

  template <typename T>
  bool operator()(const T* const k,  T* residual) const {
    residual[0] = T ( p1_.y_ -   k[0] * p1_.x_   );
    return true;
  }

 private:
  // Observations for a sample.
  // const double x_;
  // const double y_;
  point p1_;
};

void calcDirection(const std::vector<Eigen::Vector2d> &points, Eigen::Vector2d &direction)
{
    Eigen::Vector2d mean_point(0, 0);
    for (size_t i = 0; i < points.size(); i++)
    {
        mean_point(0) += points[i](0);
        mean_point(1) += points[i](1);
    }
    mean_point(0) = mean_point(0) / points.size();
    mean_point(1) = mean_point(1) / points.size();
    Eigen::Matrix2d S;
    S << 0, 0, 0, 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Matrix2d s = (points[i] - mean_point) * (points[i] - mean_point).transpose();
        S += s;
    }
    Eigen::EigenSolver<Eigen::Matrix<double, 2, 2>> es(S);
    Eigen::MatrixXcd evecs = es.eigenvectors();
    Eigen::MatrixXcd evals = es.eigenvalues();
    Eigen::MatrixXd evalsReal;
    evalsReal = evals.real();
    Eigen::MatrixXf::Index evalsMax;
    evalsReal.rowwise().sum().maxCoeff(&evalsMax); // 得到最大特征值的位置
    direction << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax);
}



int main()
{
    std::string pcdfile = "/opt/csg/slam/navs/2.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcdfile, *cloud) == -1)
    {
        std::string err = "Couldn't read file " + pcdfile;
        PCL_ERROR(err.c_str());
        return 0;
    }
    cout << "cloud size " << cloud->size() << endl;


        double k[1] = {1};
    Problem problem; // 求解对象（Problem应该是顶层的抽象类）
    problem.AddParameterBlock(k, 1); // 添加向量 x 作为参数

    // 产生随机数引擎，采用time作为种子，以确保每次运行程序都会得到不同的结果
    static default_random_engine e(time(0));
    // 产生正态分布对象
    static normal_distribution<double> n(0, 0.5);

    std::vector<Eigen::Vector2d> points;
    Eigen::Vector2d direction;
    for (size_t i = 0; i < cloud->size()*10; i++)
    {
        double random_number_x = n(e); // 把引擎作为参数，调用随机分布对象
        double random_number_y = n(e); // 把引擎作为参数，调用随机分布对象
        // Eigen::Vector2d pt( cloud->points[i].x, cloud->points[i].y  )  ;
        Eigen::Vector2d pt(1.0 * i + random_number_x , 3.0 * i + random_number_y);
        points.push_back(pt);

        point pt1( pt.x() , pt.y() );
        // point pt1( cloud->points[i].x, cloud->points[i].y  );
        CostFunction *cost_function =                                                             // 《14讲》把这个省略，直接写到AddResidualBlock中，其实一样的
          new AutoDiffCostFunction<ExponentialResidual, 1, 1>(new ExponentialResidual( pt1 )); // 前面定义的仿函数的构造函数初始化，将数据传入
        problem.AddResidualBlock(cost_function, nullptr, k);
    }
    calcDirection(points, direction);

    std::cout << direction << std::endl;
    std::cout << direction.coeffRef(0) / direction.coeffRef(1) << std::endl;
    std::cout << direction.coeffRef(1) / direction.coeffRef(0) << std::endl;


    // for (size_t i = 0; i < cloud->size(); i++)
    // {
    //     point pt( cloud->points[i].x, cloud->points[i].y );
    //     CostFunction *cost_function =                                                             // 《14讲》把这个省略，直接写到AddResidualBlock中，其实一样的
    //       new AutoDiffCostFunction<ExponentialResidual, 1, 1>(new ExponentialResidual( pt )); // 前面定义的仿函数的构造函数初始化，将数据传入
    //     problem.AddResidualBlock(cost_function, nullptr, k);
    // }

    // The options structure, which controls how the solver operates
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR ;
    // options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Run solver
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
 
    std::cout << std::fixed << std::setprecision(5) << "k is " << k[0] << "\n";

    return 1;
}