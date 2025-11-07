/******************************************************************************
 * Standalone implementation of poses interpolation for local compilation
 *****************************************************************************/

#include "poses_interpolation_standalone.h"
#include <fstream>
#include <sstream>
#include <cassert>

namespace apollo {
namespace localization {
namespace msf {

PosesInterpolation::PosesInterpolation() {}

bool PosesInterpolation::Init(const std::string &input_poses_path,
                              const std::string &ref_timestamps_path,
                              const std::string &out_poses_path,
                              const std::string &extrinsic_path) {
  this->input_poses_path_ = input_poses_path;
  this->ref_timestamps_path_ = ref_timestamps_path;
  this->out_poses_path_ = out_poses_path;
  this->extrinsic_path_ = extrinsic_path;
  if (extrinsic_path_.empty())
  {
    velodyne_extrinsic_ = Eigen::Affine3d::Identity();
    return true;
  }

  bool success = LoadExtrinsic(extrinsic_path_, &velodyne_extrinsic_);
  if (!success) {
    AERROR << "Load lidar extrinsic file failed." << std::endl;
    AERROR << "Set lidar extrinsic to identity." << std::endl;
    velodyne_extrinsic_ = Eigen::Affine3d::Identity();
  }

  return true;
}

void PosesInterpolation::DoInterpolation() {
  // Load input poses
  ::apollo::common::EigenVector3dVec input_stds;
  bool success = LoadPosesAndStds(input_poses_path_, &input_poses_, &input_stds,
                                  &input_poses_timestamps_);
  if (!success) {
    AERROR << "Failed to load poses and stds." << std::endl;
    return;
  }

  // Load pcd timestamp
  LoadPCDTimestamp();

  // Interpolation
  PoseInterpolationByTime(input_poses_, input_poses_timestamps_,
                          ref_timestamps_, ref_ids_, &out_indexes_,
                          &out_timestamps_, &out_poses_);

  // Write pcd poses
  WritePCDPoses();
}

void PosesInterpolation::LoadPCDTimestamp() {
  std::ifstream file(ref_timestamps_path_);
  if (file.is_open()) {
    unsigned int index = 1;
    double timestamp;
    double tx, ty, tz, qx, qy, qz, qw;

    while (file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
      ref_timestamps_.push_back(timestamp);
      ref_ids_.push_back(index);
      index++;
    }

    // while (file >> index >> timestamp) {
    //   ref_timestamps_.push_back(timestamp);
    // }
    file.close();
  } else {
    AINFO << "Can't open file to read: " << ref_timestamps_path_ << std::endl;
  }
}

void PosesInterpolation::WritePCDPoses() {
  std::ofstream fout;
  fout.open(out_poses_path_.c_str(), std::ofstream::out);

  if (fout.is_open()) {
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(6);
    for (size_t i = 0; i < out_poses_.size(); i++) {
      double timestamp = out_timestamps_[i];

      Eigen::Affine3d pose_tem = out_poses_[i] * velodyne_extrinsic_;
    //   Eigen::Affine3d pose_tem = velodyne_extrinsic_  * out_poses_[i];
      Eigen::Quaterniond quatd(pose_tem.linear());
      Eigen::Translation3d transd(pose_tem.translation());
      double qx = quatd.x();
      double qy = quatd.y();
      double qz = quatd.z();
      double qr = quatd.w();

      // fout << out_indexes_[i] << " " << timestamp << " " << transd.x() << " "
      fout << timestamp << " " << transd.x() << " "
           << transd.y() << " " << transd.z() << " " << qx << " " << qy << " "
           << qz << " " << qr << "\n";
    }

    fout.close();
    AINFO << "Successfully wrote " << out_poses_.size() << " poses to " 
          << out_poses_path_ << std::endl;
  } else {
    AERROR << "Can't open file to write: " << out_poses_path_ << std::endl;
  }
}


bool PosesInterpolation::LoadExtrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        AINFO << "Load extrinsic success" << std::endl; 
        AINFO << "extrinsic: " << extrinsic->translation().x() << " " << extrinsic->translation().y() << " " << extrinsic->translation().z() << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;

        return true;
      }
    }
  }
  return false;
}


bool PosesInterpolation::LoadPosesAndStds(const std::string &poses_path, 
                                         ::apollo::common::EigenAffine3dVec *poses,
                                         ::apollo::common::EigenVector3dVec *stds,
                                         std::vector<double> *timestamps) {
  std::ifstream file(poses_path);
  if (!file.is_open()) {
    AERROR << "Cannot open poses file: " << poses_path << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    
    std::istringstream iss(line);
    // unsigned int num = 1;
    double timestamp;
    double tx, ty, tz, qx, qy, qz, qw;
    double std_x, std_y, std_z;
    
    // 尝试读取格式: timestamp tx ty tz qx qy qz qw std_x std_y std_z
    if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw ) {
      timestamps->push_back(timestamp);
      
      // 创建变换矩阵
      Eigen::Quaterniond quat(qw, qx, qy, qz);
      Eigen::Translation3d trans(tx, ty, tz);
      poses->push_back(trans * quat);
      
      // 创建标准差向量
      stds->push_back(Eigen::Vector3d(std_x, std_y, std_z));
    } else {
      AWARN << "Failed to parse line: " << line << std::endl;
    }
  }
  
  file.close();
  AINFO << "Loaded " << poses->size() << " poses from " << poses_path << std::endl;
  return !poses->empty();
}

void PosesInterpolation::PoseInterpolationByTime(
    const ::apollo::common::EigenAffine3dVec &in_poses,
    const std::vector<double> &in_timestamps,
    const std::vector<double> &ref_timestamps,
    const std::vector<unsigned int> &ref_indexes,
    std::vector<unsigned int> *out_indexes, 
    std::vector<double> *out_timestamps,
    ::apollo::common::EigenAffine3dVec *out_poses) {
  out_indexes->clear();
  out_timestamps->clear();
  out_poses->clear();

  unsigned int index = 0;
  
  std::cout << "ref_timestamps.size(): " << ref_timestamps.size() << std::endl;

  for (size_t i = 0; i < ref_timestamps.size(); i++) {
    double ref_timestamp = ref_timestamps[i];
    unsigned int ref_index = ref_indexes[i];

    while (index < in_timestamps.size() &&
           in_timestamps.at(index) < ref_timestamp) {
      ++index;
    }

    // std::cout << "index: " << index << std::endl;
    // std::cout << "in_timestamps.size(): " << in_timestamps.size() << std::endl;

    if (index < in_timestamps.size()) {
      if (index >= 1) {
        double cur_timestamp = in_timestamps[index];
        double pre_timestamp = in_timestamps[index - 1];
        assert(cur_timestamp != pre_timestamp);

        double t =
            (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
        assert(t >= 0.0);
        assert(t <= 1.0);

        Eigen::Affine3d pre_pose = in_poses[index - 1];
        Eigen::Affine3d cur_pose = in_poses[index];
        Eigen::Quaterniond pre_quatd(pre_pose.linear());
        Eigen::Translation3d pre_transd(pre_pose.translation());
        Eigen::Quaterniond cur_quatd(cur_pose.linear());
        Eigen::Translation3d cur_transd(cur_pose.translation());

        Eigen::Quaterniond res_quatd = pre_quatd.slerp(1 - t, cur_quatd);

        Eigen::Translation3d re_transd;
        re_transd.x() = pre_transd.x() * t + cur_transd.x() * (1 - t);
        re_transd.y() = pre_transd.y() * t + cur_transd.y() * (1 - t);
        re_transd.z() = pre_transd.z() * t + cur_transd.z() * (1 - t);

        out_poses->push_back(re_transd * res_quatd);
        out_indexes->push_back(ref_index);
        out_timestamps->push_back(ref_timestamp);
      }
    } else {
      AWARN << "[WARN] No more poses. Exit now." << std::endl;
      break;
    }
    // ADEBUG << "Frame_id: " << i << std::endl;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
