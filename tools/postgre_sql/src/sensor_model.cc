#include "sensor_model.h"

#include <cstring>
#include <ctime>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

sensor_model::sensor_model(std::string calib_folder,
                           std::string extrinsics_folder,
                           std::string intrinsics_folder)
    : base_path(calib_folder), extrinsics_path(base_path / extrinsics_folder),
      intrinsics_path(base_path / intrinsics_folder), is_Tli_hotfix(false) {}

sensor_model::~sensor_model() {}

Eigen::Matrix4d sensor_model::get_T_lv() { return T_lv; }
Eigen::Matrix4d sensor_model::get_T_iv() {
  if (is_Tli_hotfix) {
    return T_iv_hotfix;
  } else {
    return T_iv_init;
  }
}

Eigen::Matrix4d sensor_model::get_T_li() {
  if (is_Tli_hotfix) {
    return T_li_hotfix;
  } else {
    return T_li_init;
  }
}

Eigen::Matrix4d sensor_model::get_T_vl() { return T_vl; }
Eigen::Matrix4d sensor_model::get_T_vi() {
  if (is_Tli_hotfix) {
    return T_vi_hotfix;
  } else {
    return T_vi_init;
  }
}
Eigen::Matrix4d sensor_model::get_T_il() {
  if (is_Tli_hotfix) {
    return T_il_hotfix;
  } else {
    return T_il_init;
  }
}

void sensor_model::Hotfix_T_li(const Eigen::Matrix4d &T_hotfix) {
  T_li_hotfix = T_hotfix;
  T_il_hotfix = T_hotfix.inverse();
  T_iv_hotfix = T_lv * T_il_hotfix;
  T_vi_hotfix = T_li_hotfix * T_vl;
  is_Tli_hotfix = true;
}

////////////////////////////////////////////////////////////////////

bool sensor_model::load_all() {
  //
  // 1.判断文件夹是否存在
  if (!fs::exists(this->base_path)) {
    std::cout << "Input folder does not exist: " << this->base_path
              << std::endl;
    return false;
  }

  // 2.读取所有传感器外参，外参存放在inputfolder/extrinsics文件夹中，统一的命名规则为
  // Extrinsics_XX_2_vehicle.yaml 其中XX为传感器名称
  // 判断extrinsics_path是否存在
  if (!fs::exists(this->extrinsics_path)) {
    std::cout << "Extrinsics folder does not exist: " << this->extrinsics_path
              << std::endl;
    return false;
  }

  // 遍历extrinsics_path文件夹中的所有文件
  for (const auto &entry : fs::directory_iterator(this->extrinsics_path)) {
    std::string filename = entry.path().filename();

    // Check if the file matches the naming pattern
    // "Extrinsics_XX_2_vehicle.yaml"
    if (filename.find("Extrinsics") != std::string::npos &&
        filename.find("_2_vehicle") != std::string::npos &&
        filename.find(".yaml") != std::string::npos) {
      std::string filepath = entry.path();
      std::cout << "Found extrinsics file: " << filepath << std::endl;

      // Load the YAML file (using OpenCV's FileStorage for example)
      cv::FileStorage fs(filepath, cv::FileStorage::READ);
      if (!fs.isOpened()) {
        std::cout << "Failed to open extrinsics file: " << filepath
                  << std::endl;
        continue;
      }

      // Process the extrinsics data (example: read into a cv::Mat)
      cv::Mat q, t;
      std::string sensor_name, sensor_type;
      fs["sensor_type"] >> sensor_type;
      fs["sensor_name"] >> sensor_name;
      fs["r_quaternion_wxyz"] >> q;
      fs["t_metric_xyz"] >> t;
      fs.release();

      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_eigen(q.at<double>(0), q.at<double>(1),
                                 q.at<double>(2), q.at<double>(3));
      Eigen::Vector3d t_eigen(t.at<double>(0), t.at<double>(1),
                              t.at<double>(2));
      T.block<3, 3>(0, 0) = q_eigen.toRotationMatrix();
      T.block<3, 1>(0, 3) = t_eigen;

      if (sensor_type == "camera") {
        camera_params[sensor_name] = std::make_pair(CameraIntrinsics(), T);
      } else if (sensor_type == "lidar") {
        this->T_lv = T;
        this->T_vl = T.inverse();
      } else if (sensor_type == "ins") {
        this->T_iv_init = T;
        this->T_vi_init = T.inverse();
      } else {
        std::cout << "Unknown sensor type: " << sensor_type << std::endl;
        continue;
      }
      std::cout << "Loaded extrinsics from file: " << filepath << std::endl;
      std::cout << "Sensor Name: " << sensor_name << std::endl;
      std::cout << "Sensor Type: " << sensor_type << std::endl;
      std::cout << "---------------------------------------------------"
                << std::endl;
      // You can store or process the extrinsics matrix as needed
    }
  }

  this->T_li_init = this->T_vi_init * this->T_lv;
  this->T_il_init = this->T_vl * this->T_iv_init;

  //////////////////////////////////////////////////////////////////////////////////////
  // 3.加载相机内参
  for (auto &[name, param] : camera_params) {
    // 判断extrinsics_path是否存在
    if (!fs::exists(this->intrinsics_path)) {
      std::cout << "Extrinsics folder does not exist: " << this->intrinsics_path
                << std::endl;
      return false;
    }

    // 遍历extrinsics_path文件夹中的所有文件
    for (const auto &entry : fs::directory_iterator(this->intrinsics_path)) {
      std::string filename = entry.path().filename();

      // Check if the file matches the naming pattern
      // "Intrinsic_sensor_name.yaml"
      if (filename.find("Intrinsic") != std::string::npos &&
          filename.find(name) != std::string::npos &&
          filename.find(".yaml") != std::string::npos) {

        auto &intrinsics = param.first;
        if (load_camera_intrinsics(entry.path(), intrinsics) != 0) {
          std::cout << "Failed to load camera intrinsics: " << entry.path()
                    << std::endl;
          continue;
        } else {
          std::cout << "Loaded camera intrinsics: " << entry.path()
                    << std::endl;
          camera_intrinsics_files[name] = entry.path();
          break;
        }
      }
    }
  }
  return true;
}

int sensor_model::load_camera_intrinsics(std::string yaml_file,
                                         CameraIntrinsics &internal) {
  cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return -1;
  }

  cv::Mat K, D;
  std::string model, type;

  fs["Model"] >> model;
  fs["Type"] >> type;
  fs["serial_number"] >> internal.serial_number;
  internal.width = static_cast<int>(fs["ImageWidth"]);
  internal.height = static_cast<int>(fs["ImageHeight"]);

  if (model == "brown") {
    internal.model = CameraModel::Brown;
  } else if (model == "kb8") {
    internal.model = CameraModel::Kb8;
  } else if (model == "ocam") {
    internal.model = CameraModel::Ocam;
  } else if (model == "mei") {
    internal.model = CameraModel::Mei;
  } else if (model == "rectified") {
    internal.model = CameraModel::Rectified;
  } else {
    return -1;
  }

  if (type == "pinhole") {
    internal.type = CameraType::Pinhole;
  } else if (type == "fisheye") {
    internal.type = CameraType::Fisheye;
  } else {
    return -1;
  }

  if (internal.model == CameraModel::Ocam) {
    double c = static_cast<double>(fs["ac"]);
    double d = static_cast<double>(fs["ad"]);
    double e = static_cast<double>(fs["ae"]);
    double cx = static_cast<double>(fs["center_x"]);
    double cy = static_cast<double>(fs["center_y"]);
    int pol_size = static_cast<int>(fs["length_pol"]);
    int invpol_size = static_cast<int>(fs["length_invpol"]);
    cv::Mat pol, invpol;
    fs["pol_data"] >> pol;
    fs["invpol_data"] >> invpol;

    K = cv::Mat(5, 1, CV_64F);
    D = cv::Mat(pol_size + invpol_size, 1, CV_64F);
    K.at<double>(0) = c;
    K.at<double>(1) = d;
    K.at<double>(2) = e;
    K.at<double>(3) = cx;
    K.at<double>(4) = cy;
    for (int i = 0; i < pol_size; i++) {
      D.at<double>(i) = pol.at<double>(i);
    }
    for (int i = 0; i < invpol_size; i++) {
      D.at<double>(pol_size + i) = invpol.at<double>(i);
    }

  } else {
    fs["cameraMatrix"] >> K;
    fs["distCoeffs"] >> D;
    if (fs["Xi"].empty()) {
      internal.xi = 0.0;
    } else {
      internal.xi = static_cast<double>(fs["Xi"]);
    }
  }

  K.convertTo(internal.K, CV_64F);
  D.convertTo(D, CV_64F);
  internal.D.resize(D.rows * D.cols);
  for (int i = 0; i < D.rows * D.cols; i++) {
    internal.D[i] = D.at<double>(i);
  }

  fs.release();
  return 0;
}

void sensor_model::crop_all_cameras(double cropx, double cropy) {

  // Create new folder if it does not exist
  fs::path new_folder = this->base_path / "/new_intrinsics/";
  struct stat buffer;
  if (stat(new_folder.c_str(), &buffer) != 0) {
    if (mkdir(new_folder.c_str(), 0777) != 0) {
      std::cerr << "Error creating directory: " << new_folder << std::endl;
      return;
    }
  }

  for (auto &[name, param] : camera_params) {
    CameraIntrinsics &intrinsics = param.first;

    auto new_intrinsics = intrinsics.clone();
    new_intrinsics.K.at<double>(0, 2) -= cropx; // shift cx
    new_intrinsics.K.at<double>(1, 2) -= cropy; // shift cy
    std::fill(new_intrinsics.D.begin(), new_intrinsics.D.end(),
              0.0); // reset distortion coefficients
    new_intrinsics.width =
        intrinsics.width - cropx * 2; // crop 60 pixels from each side
    new_intrinsics.height =
        intrinsics.height - cropy * 2; // crop 60 pixels from each side

    this->cropped_intrinsics[name] = new_intrinsics;

    fs::path new_file = new_folder / camera_intrinsics_files[name];
    Save_Internal_Matrix(new_file, new_intrinsics);
  }
}

void sensor_model::Save_Internal_Matrix(std::string filename,
                                        CameraIntrinsics &intrinsics) {
  // Save as YAML format as following
  //   %YAML:1.0
  // ---
  char chNewTime[64];
  time_t new_Time = std::time(NULL);
  std::strftime(chNewTime, sizeof(chNewTime), "%Y_%m_%d %H:%M:%S",
                std::localtime(&new_Time));
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "calibration_time" << (std::string)chNewTime;
  fs << "serial_number" << intrinsics.serial_number;
  fs << "ImageWidth" << intrinsics.width;
  fs << "ImageHeight" << intrinsics.height;
  fs << "Model" << intrinsics.get_model_str();
  fs << "Type"
     << (intrinsics.type == CameraType::Pinhole ? "pinhole" : "fisheye");
  fs << "cameraMatrix" << intrinsics.K;
  // Convert vector to cv::Mat for saving
  cv::Mat dist_coeffs(1, intrinsics.D.size(), CV_64F);
  for (size_t i = 0; i < intrinsics.D.size(); i++) {
    dist_coeffs.at<double>(i) = intrinsics.D[i];
  }
  fs << "distCoeffs" << dist_coeffs;
  fs.release();
}
