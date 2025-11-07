
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <experimental/filesystem>

#include <vector>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <algorithm> 

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>


using namespace std;
using namespace Eigen;

template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
     //   std::cout << "x  : " << rot(2, 1) / rot(2, 2) << std::endl;
     //   std::cout << "y  : " << -rot(2, 0) / sy << std::endl;
     //   std::cout << "z  : " << rot(1, 0) / rot(0, 0) << std::endl;
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    
    x = x * 180.0 / M_PI;
    y = y * 180.0 / M_PI;
    z = z * 180.0 / M_PI;

    // Normalize roll to be within [-180, 180]
    if (x > 180.0) x -= 360.0;
    if (x < -180.0) x += 360.0;
    // Limit yaw to be near 0
//     if (z > 90.0) z -= 180.0;
//     if (z < -90.0) z += 180.0;

    Eigen::Matrix<T, 3, 1> ang(z, y, x);
    return ang;
}


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

     return ypr * 180.0 / M_PI ;
}

#define M_PI 3.14159265358979323846
#define To_rad(x) (x * M_PI / 180.0)

void From_Martix()
{
     // T_ic = T_il * T_lc

     Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
     R_cl << 0.0161775, -0.999735, -0.0163792,
         -0.0438769, 0.0156558, -0.998914,
         0.998906, 0.0168786, -0.043612;

     Eigen::Vector3d T_cl;
     T_cl << -0.0242812, -0.0718288, -0.0921109;

     Eigen::Isometry3d T_cl_4d = Eigen::Isometry3d::Identity();
     T_cl_4d.rotate(R_cl);
     T_cl_4d.pretranslate(T_cl);
     cout << __FILE__ << ":" << __LINE__ << " T_cl_4d   " << endl
          << endl
          << T_cl_4d.matrix() << endl
          << endl
          << endl;

     // todo /////////////////////////////////////////////////////////////
     Eigen::Matrix3d R_il = Eigen::Matrix3d::Identity();

     Eigen::Vector3d T_il;
     T_il << 0.04165, 0.02326, -0.0284;

     Eigen::Isometry3d T_il_4d = Eigen::Isometry3d::Identity();
     T_il_4d.rotate(R_il);
     T_il_4d.pretranslate(T_il);
     cout << __FILE__ << ":" << __LINE__ << " T_il_4d   " << endl
          << endl
          << T_cl_4d.matrix() << endl
          << endl
          << endl;

     Eigen::Isometry3d T_ic = Eigen::Isometry3d::Identity();

     T_ic = T_il_4d * T_cl_4d.inverse();

     cout << " T_ic.matrix " << endl
          << __LINE__ << endl
          << T_ic.matrix() << endl
          << endl
          << endl;
}

void From_Quaterniond()
{
     Eigen::Vector3d p_a(0.0, 0, 0);
     Eigen::Quaterniond q_a(0.5, -0.5, 0.5, -0.5); // 1 + 0i + 0j + 0k

     Eigen::Vector3d p_b(0.017, -0.008, -0.068);
     Eigen::Quaterniond q_b( 0.9999456 , -0.0045338, 0.0039616, 0.0085177 ); // 0.7071 + 0i + 0j + 0.7071k

     Eigen::Vector3d p_ab = q_a.conjugate() * (p_b - p_a);
     Eigen::Quaterniond q_ab = q_a.conjugate() * q_b;

     std::cout << "Relative Pose p_ab  : " << p_ab.transpose() << std::endl;
     std::cout << "Relative Pose q_ab: " << q_ab.coeffs().transpose().w() << std::endl;
     std::cout << "Relative Pose q_ab: " << q_ab.coeffs().transpose() << std::endl;
     std::cout << "Relative Pose q_ab<Martix>: " << q_ab.matrix() << std::endl;
}


const std::vector<string> scan_dir_get_filename( const std::string &path )
{
     std::vector<string> filenames;
	struct dirent **entry_list;
	int count;
	int i;

	count = scandir(path.c_str(), &entry_list, 0, alphasort);
	if (count < 0)
	{
		perror("scandir");
	}

	for (i = 0; i < count; i++)
	{
		struct dirent *entry;
		entry = entry_list[i];
		// 跳过 ./ 和 ../ 两个目录
		if (i < 2)
		{
			continue;
		}
		filenames.push_back(path + std::string(entry->d_name));
		// printf("%s\n", entry->d_name);
		printf(" %s \n",  filenames.back().c_str()  );
		free(entry);
	}
	free(entry_list);
	cout << "size of pcd : " << filenames.size() << endl;
     return filenames;
}

void read_8_extrinsics2euler( const std::string extrinsics_file )
{
     // extrinsics_file = "/opt/csg/slam/navs/src/HBA/rviz_cfg/camera_v1_avia_hk_20241118.yaml";
    std::ifstream file(extrinsics_file);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return ;
    }

    Eigen::Matrix3d matrix;
    Eigen::Vector3d trans;

    std::string line;
    while (std::getline(file, line))
    {
     //     if (line.find("#") != std::string::npos)
         if (line.find("T_camera_lidar") != std::string::npos)
         {
              // Skip this line and the next three lines
              for (int i = 0; i < 3; ++i)
              {
                   std::getline(file, line);
                   // Read the matrix values
                   std::stringstream ss(line);
                   double value;
                   for (int j = 0; j < 3; ++j)
                   {
                        ss >> value;
                        matrix(i, j) = value;
                   }
                   ss >> trans(i);
              }
              // Output the matrix
          //     std::cout << "Matrix read from file:" << std::endl;
          //     std::cout << matrix << std::endl << std::endl;

              Eigen::Quaterniond quaternion(matrix);
              quaternion.normalize();
              matrix = quaternion.toRotationMatrix();

          //     std::cout << matrix << std::endl << std::endl;

              Eigen::Vector3d eu_ypr = RotMtoEuler(matrix) ;
              std::cout << "eulerAngle<RotMtoEuler>: " << eu_ypr.transpose() << std::endl;

          //     eu_ypr = R2ypr(matrix) ;
 
              std::cout << extrinsics_file << "  " ;
              std::cout << trans.transpose() << "  " ;
              std::cout << matrix.eulerAngles(0, 1, 2).transpose() * 180.0 / M_PI << std::endl;
          //     std::cout << matrix.eulerAngles(2, 1, 0).transpose() * 180.0 / M_PI << std::endl;

         }
    }

    file.close();

}

void LoadPoses(const std::string& file_path,
                  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>* poses,
                  std::vector<double>* timestamps,
                  std::vector<unsigned int>* pcd_indices) {
  poses->clear();
  timestamps->clear();
  pcd_indices->clear();

  FILE* file = fopen(file_path.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qr;
    static constexpr int kSize = 8;
//     while (fscanf(file, "%u %lf %lf %lf %lf %lf %lf %lf %lf\n", &index,
//                   &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr) == kSize) {

    while (fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf\n",  
                  &timestamp, &x, &y, &z, &qx, &qy, &qz, &qr) == kSize) {

      Eigen::Translation3d trans(Eigen::Vector3d(x, y, z));
      Eigen::Quaterniond quat(qr, qx, qy, qz);
      poses->push_back(trans * quat);
      timestamps->push_back(timestamp);
     //  pcd_indices->push_back(index);
    }
    fclose(file);
  } else {
    std::cout << "Can't open file to read: " << file_path << std::endl;
  }
}
 

int main(int argc, char **argv)
{
     Eigen::Quaterniond q1(0  , 0.7071068, 0.7071068, 0  );
     std::cout << "q1: " << q1.coeffs()  << std::endl << std::endl;
     std::cout << "q1: " << q1.inverse().coeffs()  << std::endl << std::endl;
     std::cout << "q1: " << q1.inverse().w()  << std::endl << std::endl;
     return 1;

     // rosrun tf static_transform_publisher  0.0 1.12  0.250  -0.7071068 -0.7071068  0  0 Vehicle  imu  100
     Eigen::Affine3d T_vi = Eigen::Affine3d::Identity();
     T_vi.translation() = Eigen::Vector3d(0.0 ,1.12, 0.250); // 1.12, 0, 0.54
     T_vi.rotate( Eigen::Quaterniond  ( 0 , -0.7071068, -0.7071068, 0 ) );
     std::cout << "T_vi: " << T_vi.matrix()  << std::endl << std::endl;

     // Eigen::Affine3d T_il = Eigen::Affine3d::Identity();
     // // T_il.translation() = Eigen::Vector3d(0.0696721, 1.31345 ,0.942279);
     // T_il.translation() = Eigen::Vector3d(0.0696721 , 1.31345 - 0.44 ,0.942279 - 0.6 );
     // Eigen::Quaterniond q( 0.015358, 0.00497872, 0.00177204, 0.999868 );
     // T_il.rotate(q);
     // std::cout << "T_il: " << T_il.matrix()  << std::endl << std::endl;


     // Eigen::Affine3d T_ig = Eigen::Affine3d::Identity();
     // T_ig.translation() = Eigen::Vector3d(-0.5, 0.44 ,0.6);
     // Eigen::Quaterniond q1( 1.0, 0.0, 0.0, 0.0 );
     // T_ig.rotate(q1);
     // std::cout << "T_ig: " << T_ig.matrix()  << std::endl << std::endl;
     
     // auto T_vl = (T_vi.inverse() * T_il);
     // std::cout << "T_vl.linear(): " << T_vl.linear()  << std::endl << std::endl;
     // Eigen::Quaterniond quat(T_vl.linear());

     // std::cout << "----: " << T_vl.matrix()  << std::endl << std::endl;
     // std::cout << "----: " << quat.coeffs()  << std::endl << std::endl;
     //     std::cout << "Quaternion (w, x, y, z): " 
     //          << quat.w() << ", " 
     //          << quat.x() << ", " 
     //          << quat.y() << ", " 
     //          << quat.z() << std::endl;

     // return 1;
     std::string file_path = "/mnt/nvme0n1p2/master_poses.txt";
     std::cout << "file_path: " << file_path << std::endl;

     std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> pcd_poses;
     std::vector<double> timestamps;
     std::vector<unsigned int> pcd_indices;
     LoadPoses( file_path, &pcd_poses, &timestamps, &pcd_indices);
     std::cout << "pcd_poses size: " << pcd_poses.size() << std::endl;
     for (size_t i = 0; i < pcd_poses.size(); i++)
     {
          // const Eigen::Affine3d T_wl = pcd_poses[i] * T_vi * T_il;
          // const Eigen::Affine3d T_wl = pcd_poses[i] * T_ig.inverse() * T_il;


          // const Eigen::Affine3d T_wc = pcd_poses[i] * T_vi.inverse();
          // Eigen::Quaterniond cur_quat  = (Eigen::Quaterniond)T_wc.linear();

          // T_wc.translation().x() = pcd_poses[i].translation().y();
          // T_wc.translation().y() = pcd_poses[i].translation().x();
          // T_wc.translation().z() = -pcd_poses[i].translation().z();

          // std::cout 
          // //   << i << " "
          //   << std::fixed << std::setprecision(3)
          //   << timestamps[i]   << " "
          //   << T_wc.translation().x()  << " "
          //   << T_wc.translation().y()  << " "
          //   << T_wc.translation().z()  << " "
          //   << std::setprecision(6)
          //   << cur_quat.x()  << " "
          //   << cur_quat.y()  << " "
          //   << cur_quat.z()  << " "
          //   << cur_quat.w() << std::endl;

          // NED到ENU的坐标转换
          const Eigen::Affine3d& ned_pose = pcd_poses[i];
          
          // 创建ENU坐标系下的pose
          Eigen::Affine3d enu_pose = Eigen::Affine3d::Identity();
          
          // 位置转换: ENU(x,y,z) = (NED_y, NED_x, -NED_z)
          enu_pose.translation().x() = ned_pose.translation().x();  // ENU_x = NED_y
          enu_pose.translation().y() = ned_pose.translation().y();  // ENU_y = NED_x
          enu_pose.translation().z() = ned_pose.translation().z(); // ENU_z = -NED_z
          
          // 姿态转换: NED到ENU的旋转矩阵
          Eigen::Matrix3d R_ned_to_enu;
          R_ned_to_enu << 0, 1, 0,
                          1, 0, 0,
                          0, 0, -1;
          
          // 计算转换前的欧拉角（NED）
          Eigen::Matrix3d ned_rotation = ned_pose.linear();
          Eigen::Vector3d euler_ned = RotMtoEuler(ned_rotation);
          
          // 应用坐标系转换到旋转部分
          enu_pose.linear() = R_ned_to_enu * ned_pose.linear()  ;
          
          // 计算转换后的欧拉角（ENU）
          Eigen::Matrix3d enu_rotation = enu_pose.linear();
          Eigen::Vector3d euler_enu = RotMtoEuler(enu_rotation);
          // Eigen::Vector3d euler_enu_maunal ( - euler_ned.x(), euler_ned.y(), euler_ned.z());
          Eigen::Vector3d euler_enu_maunal (  euler_ned.x(), - euler_ned.y(), - euler_ned.z());
          
          // 将欧拉角(度)转换为四元数 - ZYX顺序 (yaw, pitch, roll)
          double yaw_rad = euler_enu.x() * M_PI / 180.0;
          double pitch_rad = euler_enu.y() * M_PI / 180.0;
          double roll_rad = euler_enu.z() * M_PI / 180.0;
          
          // ZYX欧拉角转四元数
          Eigen::AngleAxisd yaw_angle(yaw_rad, Eigen::Vector3d::UnitZ());
          Eigen::AngleAxisd pitch_angle(pitch_rad, Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd roll_angle(roll_rad, Eigen::Vector3d::UnitX());
          
          Eigen::Quaterniond cur_quat = yaw_angle * pitch_angle * roll_angle;
          
          // 打印欧拉角变化（每10个点打印一次避免输出过多）
          if (i % 10 == 0) {
              std::cout << "Frame " << i << ":" << std::endl;
              std::cout << "  NED Euler (YPR): " << euler_ned.transpose() << " deg" << std::endl;
              std::cout << "  ENU Euler (YPR): " << euler_enu.transpose() << " deg" << std::endl;
              std::cout << "  ENU Euler (YPR): " << euler_enu_maunal.transpose() << " deg" << std::endl;
              std::cout << "  Difference: " << (euler_enu - euler_ned).transpose() << " deg" << std::endl;
          }
          
          // 提取四元数
          // Eigen::Quaterniond cur_quat(enu_pose.linear());
          cur_quat.normalize();

          // 输出TUM格式: timestamp x y z qx qy qz qw
          std::cout 
            << std::fixed << std::setprecision(6)
            << timestamps[i]   << " "
            << enu_pose.translation().x()  << " "
            << enu_pose.translation().y()  << " "
            << enu_pose.translation().z()  << " "
            << cur_quat.x()  << " "
            << cur_quat.y()  << " "
            << cur_quat.z()  << " "
            << cur_quat.w() << std::endl;


     }

     // From_Martix();
     return 1;

     // From_Quaterniond();
     // return 1;
     
     // read_8_extrinsics2euler();

     std::string extrinsics_file = "/opt/csg/slam/navs/TF70/";
    if (argc == 2) {
        std::cerr << "Usage: " << argv[0] << " <extrinsics_file_path>" << std::endl;
        extrinsics_file = argv[1];
    }

     std::cout << "extrinsics_file: " << extrinsics_file  << std::endl << std::endl;

     auto fiels_this_dir = scan_dir_get_filename(extrinsics_file);

     for ( const auto item : fiels_this_dir )
     {
         read_8_extrinsics2euler( item );
     }
     
     return 1;


     Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
     R_cl <<  0.9995320,  0.0001047,  0.0305908,
  -0.0004363,  0.9999412,  0.0108332,
  -0.0305879, -0.0108415,  0.9994733 ;

     //    R_cl <<   0.00,   -1.0,  -0.0 ,
     //              0.00,   0.0,   -1.0 ,
     //              1.00 ,  0.00 , 0.00 ;

     std::cout << "R_cl: " << R_cl  << std::endl << std::endl;

     Eigen::Vector3d eu_ypr = RotMtoEuler(R_cl) ;
     std::cout << "eulerAngle<R2ypr>: " << eu_ypr.transpose()  << std::endl;

     //   camera 坐标系 先绕 x 转90  绕y 转0   ，绕z转90
     std::cout << "eulerAngles<eigen012>: " << R_cl.eulerAngles(0, 1, 2).transpose() * 180.0 / M_PI  << std::endl<< std::endl;
     std::cout << "eulerAngles<eigen210>: " << R_cl.eulerAngles(2, 1, 0).transpose() * 180.0 / M_PI  << std::endl<< std::endl;

     return 1;


         
     string pcd_file = "125";
     // std::cout << "Relative Pose Quaternion  w: " << 3*pcd_file  << std::endl;

     Eigen::Vector3d eulerAngle(-1.57079632679, 0, -1.57079632679);
     Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitX()));
     Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
     Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitZ()));

     Eigen::Matrix3d rotation_matrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
     //   Eigen::Matrix3d rotation_matrix = ( rollAngle* pitchAngle * yawAngle).toRotationMatrix();

     Eigen::Isometry3d T_rc = Eigen::Isometry3d::Identity();
     T_rc.rotate(rotation_matrix);

     Eigen::Vector3d trans_mat(0, 0, 0);
     T_rc.pretranslate(trans_mat);

     cout << __FILE__ << ":" << __LINE__ << " T_rc " << endl
          << endl
          << T_rc.matrix() << endl
          << endl
          << endl;

     Eigen::Vector3d eulerAngle_rl(0.0156202, 0.00180632, -0.00919182);
     Eigen::AngleAxisd rollAngle_rl(AngleAxisd(eulerAngle_rl(0), Vector3d::UnitX()));
     Eigen::AngleAxisd pitchAngle_rl(AngleAxisd(eulerAngle_rl(1), Vector3d::UnitY()));
     Eigen::AngleAxisd yawAngle_rl(AngleAxisd(eulerAngle_rl(2), Vector3d::UnitZ()));

     Eigen::Matrix3d rotation_matrix_rl = (yawAngle_rl * pitchAngle_rl * rollAngle_rl).toRotationMatrix();

     Eigen::Isometry3d T_rl = Eigen::Isometry3d::Identity();
     T_rl.rotate(rotation_matrix_rl);

     Eigen::Vector3d trans_mat_rl(0.0190991 ,-0.00377715 ,-0.0624914 );
     T_rl.pretranslate(trans_mat_rl);

     cout << __FILE__ << ":" << __LINE__ << " T_rl: " << endl
          << endl
          << T_rl.matrix() << endl
          << endl
          << endl;

     auto T_cl = T_rc.inverse() * T_rl;

     cout << __FILE__ << ":" << __LINE__ << " T_cl: " << endl
          << endl
          << T_cl.matrix() << endl
          << endl
          << endl;

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
