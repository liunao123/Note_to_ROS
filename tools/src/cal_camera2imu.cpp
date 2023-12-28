
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;
}

int main(int argc, char **argv)
{
  // 求逆矩阵
//   Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
//   rot_mat << -0.00207836,-0.999688,-0.0248923,
//                  -0.0127348,0.0249168,-0.999608,
//                  0.999917,-0.00176055,-0.0127826;

//   Eigen::Vector3d trans_mat;
//   trans_mat <<  -0.0407803 , 0.0531845 ,-0.051548 ; // Horizon

//   Eigen::Isometry3d T_mat = Eigen::Isometry3d::Identity();
//   T_mat.prerotate( rot_mat );
//   T_mat.translate( trans_mat );
//   cout << __FILE__ << ":" << __LINE__ << " T_mat  inverse() " << endl << endl
//        << T_mat.inverse().matrix() << endl
//        << endl << endl ;
//    return 1;


  // T_lidar2imu
  Eigen::Matrix3d r_imu_lidar = Eigen::Matrix3d::Identity();
//   r_imu_lidar <<  0.000745473  , -0.999987 , -0.00504666, 
//           0.0247745 , 0.00506358 , -0.99968,
//           0.999693 , 0.000620206 , 0.024778;

  Eigen::Vector3d t_imu_lidar;
  // t_imu_lidar <<  0.05512, 0.02226, 0.0297; // Horizon
  t_imu_lidar <<  0.014, -0.012, -0.015 ; 

  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  T_imu_lidar.rotate( r_imu_lidar );
  T_imu_lidar.pretranslate( t_imu_lidar );

  // T_lidar2camera
  Eigen::Matrix3d r_camera_lidar = Eigen::Matrix3d::Identity();
  r_camera_lidar <<    0.00343916 , 0.99997599 ,-0.00601628,
 -0.00100545 , 0.00601977 , 0.99998138 ,
  0.99999358 ,-0.00343304 , 0.00102613 ;

  Eigen::Vector3d t_camera_lidar;
  t_camera_lidar << -0.04746142 ,  0.00916453 , -0.04904955;
   

  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
  T_camera_lidar.rotate( r_camera_lidar );
  T_camera_lidar.pretranslate( t_camera_lidar );

      cout << "  T_camera_lidar " << endl << __LINE__  << endl
       << T_camera_lidar.matrix() << endl
       << endl << endl ;

  Eigen::Isometry3d T_ic = Eigen::Isometry3d::Identity();
//   T_ic = T_imu_lidar * T_camera_lidar.inverse();
  // T_ic =  T_camera_lidar.inverse() *  T_imu_lidar ;
  T_ic =  T_camera_lidar *  T_imu_lidar ;

  cout << " T_ic.matrix " << endl << __LINE__ << endl
       << T_ic.matrix() << endl
       << endl << endl ;
  

    cout << " T_ic.matrix " << endl << __LINE__ << endl
       << T_ic.inverse().matrix() << endl
       << endl << endl ;

  return 1;



//   Eigen::Matrix3d rotation = T_ic.rotation();
//   Eigen::Quaterniond qua(rotation);
//   qua.normalize();
  
//   cout << "qua.toRotationMatrix() " << endl
//        << qua.toRotationMatrix() << endl
//        << endl;

//   cout << "qua.toRotationMatrix() " << endl
//        << qua.toRotationMatrix() * qua.toRotationMatrix().transpose() << endl
//        << endl;

//   return 0;

}