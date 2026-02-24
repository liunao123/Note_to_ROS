#pragma once

#include <filesystem> // C++17
#include <getopt.h>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Eigen>

namespace fs = std::filesystem;

struct Triangle {
  int v0, v1, v2; // 对应点云中的三个顶点索引
  Triangle(int v0_, int v1_, int v2_) : v0(v0_), v1(v1_), v2(v2_) {}
};

inline bool FitPlaneFromTriangle(Eigen::Vector3f p0, Eigen::Vector3f p1,
                                 Eigen::Vector3f p2,
                                 Eigen::Vector4f &plane_coeffs) {
  // 验证输入点是否有效
  if (!std::isfinite(p0.x()) || !std::isfinite(p0.y()) ||
      !std::isfinite(p0.z()) || !std::isfinite(p1.x()) ||
      !std::isfinite(p1.y()) || !std::isfinite(p1.z()) ||
      !std::isfinite(p2.x()) || !std::isfinite(p2.y()) ||
      !std::isfinite(p2.z())) {
    throw std::runtime_error("输入点包含无效值(NaN或Inf)");
    return false;
  }

  // 计算两个边向量
  Eigen::Vector3f v1 = p1 - p0; // 边向量1: p0 -> p1
  Eigen::Vector3f v2 = p2 - p0; // 边向量2: p0 -> p2

  // 检查三点是否共线（叉积的模长接近0）
  Eigen::Vector3f normal = v1.cross(v2);
  float normal_length = normal.norm();

  if (normal_length < 1e-10f) {
    std::cout << normal_length << std::endl;
    throw std::runtime_error("三个点共线，无法确定唯一平面");
    return false;
  }

  // 归一化法向量
  normal.normalize();

  // 计算平面方程 ax + by + cz + d = 0
  // 其中 (a, b, c) 是法向量，d 通过点p0计算
  float a = normal.x();
  float b = normal.y();
  float c = normal.z();
  float d = -(a * p0.x() + b * p0.y() + c * p0.z());

  // 返回平面参数向量 [a, b, c, d]
  plane_coeffs << a, b, c, d;

  return true;
}

inline bool IsInTriangle(const Eigen::Vector3f &p, const Eigen::Vector3f &p0,
                         const Eigen::Vector3f &p1, const Eigen::Vector3f &p2,
                         const Eigen::Vector4f &plane,
                         Eigen::Vector3f &projFoot, float threshold) {
  // 验证输入参数
  if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
    return false;
  }

  // 提取平面法向量和距离参数
  Eigen::Vector3f normal(plane[0], plane[1], plane[2]);
  float d = plane[3];

  // 计算点p到平面的距离
  float distance = std::abs(normal.dot(p) + d) / normal.norm();

  // 如果距离超过阈值，直接返回false
  if (distance > threshold) {
    return false;
  }

  // 计算点p在平面上的垂足投影点
  projFoot = p - (normal.dot(p) + d) * normal;

  // 使用重心坐标判断投影点是否在三角形内
  // 计算三角形的两个边向量
  Eigen::Vector3f v0 = p2 - p0;       // p0 -> p2
  Eigen::Vector3f v1 = p1 - p0;       // p0 -> p1
  Eigen::Vector3f v2 = projFoot - p0; // p0 -> projFoot

  // 计算点积
  float dot00 = v0.dot(v0);
  float dot01 = v0.dot(v1);
  float dot02 = v0.dot(v2);
  float dot11 = v1.dot(v1);
  float dot12 = v1.dot(v2);

  // 计算重心坐标
  float inv_denom = 1.0f / (dot00 * dot11 - dot01 * dot01);
  float u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
  float v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

  // 检查点是否在三角形内
  // 重心坐标条件：u >= 0, v >= 0, u + v <= 1
  return (u >= -1e-6f) && (v >= -1e-6f) && (u + v <= 1.0f + 1e-6f);
}

inline int pointClassify(const Eigen::Vector3f &p, const Eigen::Vector3f &p0,
                         const Eigen::Vector3f &p1, const Eigen::Vector3f &p2,
                         const Eigen::Vector4f &plane,
                         Eigen::Vector3f &projFoot, float threshold) {
  // 验证输入参数
  if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
    return 0;
  }

  // 提取平面法向量和距离参数
  Eigen::Vector3f normal(plane[0], plane[1], plane[2]);
  float d = plane[3];

  // 计算点p到平面的距离
  float distance = std::abs(normal.dot(p) + d) / normal.norm();

  // 计算点p在平面上的垂足投影点
  projFoot = p - (normal.dot(p) + d) * normal;

  // 使用重心坐标判断投影点是否在三角形内
  // 计算三角形的两个边向量
  Eigen::Vector3f v0 = p2 - p0;       // p0 -> p2
  Eigen::Vector3f v1 = p1 - p0;       // p0 -> p1
  Eigen::Vector3f v2 = projFoot - p0; // p0 -> projFoot

  // 计算点积
  float dot00 = v0.dot(v0);
  float dot01 = v0.dot(v1);
  float dot02 = v0.dot(v2);
  float dot11 = v1.dot(v1);
  float dot12 = v1.dot(v2);

  // 计算重心坐标
  float inv_denom = 1.0f / (dot00 * dot11 - dot01 * dot01);
  float u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
  float v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

  // 检查点是否在三角形内
  // 重心坐标条件：u >= 0, v >= 0, u + v <= 1
  bool flag1 = (u >= -1e-6f) && (v >= -1e-6f) && (u + v <= 1.0f + 1e-6f);
  bool flag2 = (distance <= threshold || distance > 2.0f);

  if (flag1 && flag2) {
    return 3; // 在三角形内且距离阈值内 （保留）
  } else if (flag1 && !flag2) {
    return 1; // 在三角形内但距离阈值外 (剔除)
  } else {
    return 2; // 不在三角形内（保留）
  }
}