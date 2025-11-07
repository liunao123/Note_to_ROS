/******************************************************************************
 * Standalone version of poses interpolation for local compilation
 *****************************************************************************/

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

// 简化的日志宏定义
#define AINFO std::cout << "[INFO] "
#define AERROR std::cerr << "[ERROR] "
#define AWARN std::cout << "[WARN] "
#define ADEBUG std::cout << "[DEBUG] "

// 定义Eigen类型别名
namespace apollo {
namespace common {

using EigenAffine3dVec = std::vector<Eigen::Affine3d>;
using EigenVector3dVec = std::vector<Eigen::Vector3d>;

}  // namespace common
}  // namespace apollo

namespace apollo {
namespace localization {
namespace msf {

class PosesInterpolation {
 public:
  PosesInterpolation();
  bool Init(const std::string &input_poses_path,
            const std::string &ref_timestamps_path,
            const std::string &out_poses_path,
            const std::string &extrinsic_path);
  void DoInterpolation();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void LoadPCDTimestamp();
  void WritePCDPoses();
  void PoseInterpolationByTime(
      const ::apollo::common::EigenAffine3dVec &in_poses,
      const std::vector<double> &in_timestamps,
      const std::vector<double> &ref_timestamps,
      const std::vector<unsigned int> &ref_indexes,
      std::vector<unsigned int> *out_indexes,
      std::vector<double> *out_timestamps,
      ::apollo::common::EigenAffine3dVec *out_poses);

  // 简化的文件加载函数
  bool LoadExtrinsic(const std::string &extrinsic_path, Eigen::Affine3d *extrinsic);
  bool LoadPosesAndStds(const std::string &poses_path, 
                       ::apollo::common::EigenAffine3dVec *poses,
                       ::apollo::common::EigenVector3dVec *stds,
                       std::vector<double> *timestamps);

 private:
  std::string input_poses_path_;
  std::string ref_timestamps_path_;
  std::string out_poses_path_;
  std::string extrinsic_path_;

  Eigen::Affine3d velodyne_extrinsic_;

  ::apollo::common::EigenAffine3dVec input_poses_;
  std::vector<double> input_poses_timestamps_;

  std::vector<double> ref_timestamps_;
  std::vector<unsigned int> ref_ids_;

  std::vector<unsigned int> out_indexes_;
  std::vector<double> out_timestamps_;
  ::apollo::common::EigenAffine3dVec out_poses_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
