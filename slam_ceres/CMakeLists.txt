cmake_minimum_required(VERSION 3.5)
project(test_ceres)

## Compile as C++17, required for Ceres 2.3.0
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# 制定Ceres版本
SET(CMAKE_PREFIX_PATH "/root/Downloads/Ceres-2.1.0/" ${CMAKE_PREFIX_PATH})
find_package(Ceres REQUIRED)

# Find glog
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)


include_directories(
  include
  common
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARIES})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


# if (GFLAGS)
  add_executable(pose_graph_2d
    pose_graph_2d/angle_manifold.h
    pose_graph_2d/normalize_angle.h
    pose_graph_2d/pose_graph_2d.cc
    pose_graph_2d/pose_graph_2d_error_term.h
    pose_graph_2d/types.h)
  target_link_libraries(pose_graph_2d PRIVATE Ceres::ceres gflags ${GLOG_LIBRARIES})
# endif (GFLAGS)

add_executable(pose_graph_3d pose_graph_3d/pose_graph_3d.cc)
target_link_libraries(pose_graph_3d PRIVATE Ceres::ceres gflags ${GLOG_LIBRARIES})
