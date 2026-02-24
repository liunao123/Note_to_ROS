#pragma once

#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <thread>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

// BOOST
#include <boost/format.hpp>

// eigen
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

// opencv
#include <opencv2/opencv.hpp>

const double gravity_ = 9.80665;

const double rad2deg = 180.0 / M_PI;
const double deg2rad = M_PI / 180.0;

struct Point {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    Point,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

typedef Point PointType;

// Hcinspvatzcb
struct chcnav_devpvt {
  double timestamp;
  double latitude; //设备(ins)的经纬高
  double longitude;
  double altitude;
  Eigen::Vector3f position_stdev;
  float undulation;

  float roll;
  float pitch;
  float yaw;
  Eigen::Vector3f euler_stdev;

  float speed;    //地面速度
  float heading;  //速度航向，正北为0，顺时针为正，0°~360°
  float heading2; //双天线航向，正北为0，顺时针为正，0°~360°

  Eigen::Vector3f enu_velocity; // ENU坐标系下速度
  Eigen::Vector3f enu_velocity_stdev;

  Eigen::Vector3f vehicle_angular_velocity; //车辆坐标系下，去零偏 deg/s
  Eigen::Vector3f vehicle_linear_velocity; //车辆坐标系下速度
  Eigen::Vector3f
      vehicle_linear_acceleration; // 车辆坐标系下，去零偏，不补偿重力
  Eigen::Vector3f
      vehicle_linear_acceleration_without_g; // 车辆坐标系下，去零偏，补偿重力

  // 与/chcnav/imu_raw topic一致 注意：/chcnav/imu_raw中单位为rad/s , m/s^2
  Eigen::Vector3f raw_angular_velocity; //设备坐标系下，不去零偏, 单位:deg/s
  Eigen::Vector3f raw_acceleration; //设备坐标系下，不去零偏，不补偿重力, 单位:g
};

struct ImuMeas {
  double stamp;
  double dt; // defined as the difference between the current and the previous
             // measurement
  Eigen::Vector3f ang_vel;   // rad/s
  Eigen::Vector3f lin_accel; // m/s^2
};

struct Odometry {
  double timestamp;
  Eigen::Vector3d position_lla; // in LLA system
  Eigen::Matrix3d pose;         // in ENU system
  Eigen::Vector3d enu_velocity;
  double heading;
  double speed;
  ImuMeas imu_raw;
  ImuMeas imu_meas;
};

// camera type
enum class CameraType { Pinhole = 0, Fisheye = 1 };

// camera model
enum class CameraModel { Brown = 0, Kb8 = 1, Ocam = 2, Mei = 3, Rectified = 4 };

// camera intrinsics
struct CameraIntrinsics {
  cv::Mat K;             // for ocam model, K: c, d, e, cx, cy (5)
  std::vector<double> D; // for ocam model, D: pol invpol (5+13)
  CameraModel model;
  CameraType type;
  int width, height;
  double xi;                 // only for mei model
  std::string serial_number; // camera serial number

  std::string get_model_str() {
    switch (model) {
    case CameraModel::Brown:
      return "brown";
    case CameraModel::Kb8:
      return "kb8";
    case CameraModel::Ocam:
      return "ocam";
    case CameraModel::Mei:
      return "mei";
    case CameraModel::Rectified:
      return "rectified";
    default:
      return "unknown";
    }
  }

  CameraIntrinsics clone() {
    CameraIntrinsics clone;
    clone.K = K.clone();
    clone.D = D;
    clone.model = model;
    clone.type = type;
    clone.width = width;
    clone.height = height;
    clone.xi = xi;
    clone.serial_number = serial_number;
    return clone;
  }

  CameraIntrinsics get_opt() {
    CameraIntrinsics new_int = this->clone();

    // old parameters
    int w = new_int.width;
    int h = new_int.height;
    double fx = new_int.K.at<double>(0, 0);
    double fy = new_int.K.at<double>(1, 1);
    double cx = new_int.K.at<double>(0, 2);
    double cy = new_int.K.at<double>(1, 2);

    std::vector<cv::Point2f> src_points[4];
    std::vector<cv::Point2f> dst_points[4];

    // boundary points of original image
    for (int i = 0; i < w; i++) {
      src_points[0].emplace_back(static_cast<float>(i), 0.0f);
      src_points[2].emplace_back(static_cast<float>(i), h - 1.f);
    }
    for (int i = 0; i < h; i++) {
      src_points[1].emplace_back(0.0f, static_cast<float>(i));
      src_points[3].emplace_back(w - 1.f, static_cast<float>(i));
    }
    for (int i = 0; i < 4; i++) {
      cv::undistortPoints(src_points[i], dst_points[i], new_int.K, new_int.D);
    }

    // calc max rect on undistorted image
    cv::Point2f p_extrema[4];
    p_extrema[0] = dst_points[0][0];
    for (int i = 0; i < dst_points[0].size(); i++) {
      if (p_extrema[0].y < dst_points[0][i].y) {
        p_extrema[0] = dst_points[0][i];
      }
    }

    p_extrema[1] = dst_points[1][0];
    for (int i = 0; i < dst_points[1].size(); i++) {
      if (p_extrema[1].x < dst_points[1][i].x) {
        p_extrema[1] = dst_points[1][i];
      }
    }

    p_extrema[2] = dst_points[2][0];
    for (int i = 0; i < dst_points[2].size(); i++) {
      if (p_extrema[2].y > dst_points[2][i].y) {
        p_extrema[2] = dst_points[2][i];
      }
    }

    p_extrema[3] = dst_points[3][0];
    for (int i = 0; i < dst_points[3].size(); i++) {
      if (p_extrema[3].x > dst_points[3][i].x) {
        p_extrema[3] = dst_points[3][i];
      }
    }

    // set pp to center of new image
    // and keep the image size
    double new_cx = (new_int.width - 1) / 2.0;
    double new_cy = (new_int.height - 1) / 2.0;

    // calculate new focal length
    double zoom_fct[4];
    zoom_fct[0] = -new_cy / fy / p_extrema[0].y;
    zoom_fct[1] = -new_cx / fx / p_extrema[1].x;
    zoom_fct[2] = (h - 1 - new_cy) / fy / p_extrema[2].y;
    zoom_fct[3] = (w - 1 - new_cx) / fx / p_extrema[3].x;

    double max_zoom_fct = zoom_fct[0];
    for (int i = 1; i < 4; i++) {
      if (zoom_fct[i] > max_zoom_fct) {
        max_zoom_fct = zoom_fct[i];
      }
    }

    // update camera intrinsics
    new_int.K.at<double>(0, 0) = fx * max_zoom_fct;
    new_int.K.at<double>(1, 1) = fy * max_zoom_fct;
    new_int.K.at<double>(0, 2) = new_cx;
    new_int.K.at<double>(1, 2) = new_cy;

    return new_int;
  }

  CameraIntrinsics get_resize(float scale) {
    CameraIntrinsics new_int = this->clone();
    new_int.K.at<double>(0, 0) *= scale;
    new_int.K.at<double>(1, 1) *= scale;
    new_int.K.at<double>(0, 2) *= scale;
    new_int.K.at<double>(1, 2) *= scale;
    new_int.width *= scale;
    new_int.height *= scale;
    return new_int;
  }

  CameraIntrinsics get_zoom(float scale) {
    CameraIntrinsics new_int = this->clone();
    new_int.K.at<double>(0, 0) *= scale;
    new_int.K.at<double>(1, 1) *= scale;
    return new_int;
  }

  CameraIntrinsics get_center_crop(size_t cropx, size_t cropy) {
    CameraIntrinsics new_int = this->clone();
    new_int.K.at<double>(0, 2) -= cropx;
    new_int.K.at<double>(1, 2) -= cropy;
    new_int.width -= cropx * 2;
    new_int.height -= cropy * 2;
    return new_int;
  }
};