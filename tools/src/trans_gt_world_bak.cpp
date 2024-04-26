
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

  Eigen::Isometry3d r_camera_lidar = Eigen::Isometry3d::Identity();
  r_camera_lidar(0, 3) = -0.293656;
  r_camera_lidar(1, 3) = -0.012288;
  r_camera_lidar(2, 3) = -0.273095;

  cout << " r_camera_lidar" << endl
       << __LINE__ << endl
       << r_camera_lidar.matrix() << endl
       << endl
       << endl;

  ifstream infile_lio;
  const char *delim = " ";
  char buf[1024];

  string work_dir = "/opt/csg/slam/navs/livos/result/eee01/";

  string file_temp = work_dir + "trans_pose_leika.txt";
  std::ofstream out_pose_file;//(file_temp, ios::out);
  out_pose_file.open(file_temp, ios::out);

  // infile_lio.open(work_dir + "r3live_body_path.txt", ios::in);
  infile_lio.open(work_dir + "leika.txt", ios::in);
  if (!infile_lio.is_open())
  {
    std::cout << __LINE__ << ":" << __LINE__ << std::endl;
    return 0;
  }
  while (infile_lio.getline(buf, sizeof(buf)))
  {
    char *p;
    p = strtok(buf, delim);
    std::vector<double> item_8;
    while (p)
    {
      std::string item(p);
      item_8.push_back(stold(item));
      p = strtok(NULL, delim);
    }

    Eigen::Quaterniond quaternion(item_8[4], item_8[5], item_8[6], item_8[7]);
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    Eigen::Vector3d t_imu_lidar(item_8[1], item_8[2], item_8[3]);

    Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
    T_imu_lidar.pretranslate(t_imu_lidar);
    T_imu_lidar.rotate(rotation_matrix);

    Eigen::Isometry3d new_pose = T_imu_lidar * r_camera_lidar;

    Eigen::Quaterniond trans_que(new_pose.rotation());

    // 往上面的文件里写 打开
    std::cout << __LINE__ << ":" << new_pose(0, 3) << std::endl;

    out_pose_file.setf(ios::fixed, ios::floatfield);
    out_pose_file.precision(3);
    out_pose_file << item_8[0] << " ";
    out_pose_file.precision(5);

    out_pose_file
        << new_pose(0, 3) << " "
        << new_pose(1, 3) << " "
        << new_pose(2, 3) << " "
        << trans_que.x() << " "
        << trans_que.y() << " "
        << trans_que.z() << " "
        << trans_que.w() << endl;
  }
  infile_lio.close();
  out_pose_file.close();

  return 2;

}