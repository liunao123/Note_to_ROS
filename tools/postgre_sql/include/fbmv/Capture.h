/**
 * This file is part of monocular after sale calibration
 *
 * author: ruoyu.pang
 * date: 2024/03/25
 */

#pragma once

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

using std::string;
using std::vector;

namespace fbmv_calib {

enum class CameraModel { Pinhole = 0, Fisheye = 1, Rectified = 2, Mei = 3 };

struct CameraIntrinsics {
  cv::Mat K;
  cv::Mat D; // pinhole:k1 k2 p1 p2 k3 (k4 k5 k6) fisheye:k1 k2 k3 k4
  float xi;  // Mei model
  CameraModel model;
  int width, height;

  CameraIntrinsics clone() const {
    CameraIntrinsics ret;
    ret.K = K.clone();
    ret.D = D.clone();
    ret.model = model;
    ret.xi = xi;
    ret.width = width;
    ret.height = height;
    return ret;
  }

  CameraIntrinsics()
      : xi(0.f), model(CameraModel::Rectified), width(0), height(0) {}
};

// sync frame ,  for offline demo
struct SyncFrame {
  size_t serialnum; // invalid

  std::unordered_map<size_t, double> timestamps_img;
  std::unordered_map<size_t, cv::Mat> images;
  std::unordered_map<size_t, CameraIntrinsics> intrinsics;
  std::unordered_map<size_t, Eigen::Matrix4f> degisn_Tcv;

  double timestamp_odom;
  // double angular_velocity[3];     // x,y,z
  // double linear_velocity[3];      // x,y,z
  // double linear_acceleration[3];  // x,y,z
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d linear_acceleration;
  Eigen::Matrix4d odom_pose;
  Eigen::Vector3d offset_utm;
  // Eigen::Vector3d odom_t;
  // Eigen::Quaterniond odom_r;
};

} // namespace fbmv_calib