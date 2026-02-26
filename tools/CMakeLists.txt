cmake_minimum_required(VERSION 2.8)

project(rotation_pcd_to_horizon_pcl)

SET(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL  REQUIRED)
# find_package(PCL 1.2 REQUIRED)
message(STATUS "PCL version: ${PCL_VERSION}")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
include_directories(   ${EIGEN3_INCLUDE_DIR}  )

include_directories( ${PCL_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(libLAS REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)


add_executable(select_point_with_range src/select_point_with_range.cpp)
target_include_directories(select_point_with_range PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(select_point_with_range
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${libLAS_LIBRARIES}
    ${PROJ_LIBRARIES}
    Eigen3::Eigen
)


SET(src_file grid_pts_squre)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} ${OpenCV_LIBS} ${GeographicLib_LIBRARIES} )


SET(src_file get_session_data_and_NonUniformSampling)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
# yaml-cpp

SET(src_file box_filter)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} )

# 添加可执行文件
add_executable(poses_interpolator
    src/poses_interpolator.cc
    src/poses_interpolation_standalone.cc
)

# 链接库
target_link_libraries(poses_interpolator
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
)


# Create executable
add_executable(get_session_data_manfu_3d src/get_session_data_manfu_3d.cpp)

# Link libraries
target_link_libraries(get_session_data_manfu_3d ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${libLAS_LIBRARIES}
    ${PROJ_LIBRARIES}
    yaml-cpp
)


# No additional libraries needed, just standard C++17 filesystem

# Set compile flags
# target_compile_options(pointcloud_transform PRIVATE ${YAML_CPP_CFLAGS_OTHER})

SET(src_file trans_pts)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} Eigen3::Eigen)
target_compile_options(${src_file}  PRIVATE ${YAML_CPP_CFLAGS_OTHER})


add_executable (octree src/octree.cpp )
target_link_libraries (octree   ${PCL_LIBRARIES} )


add_executable(test_opencv src/test_opencv.cpp)
target_link_libraries(test_opencv  ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES}  )

add_executable(test_fisheye src/test_fisheye.cpp)
target_link_libraries(test_fisheye   ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES}  )

add_executable (remove_outlier_pt src/remove_outlier_pt.cpp )
target_link_libraries (remove_outlier_pt   ${PCL_LIBRARIES} )

SET(src_file down_filtered_pcd)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  ${PCL_LIBRARIES} )

# add_executable (mls src/mls.cpp )
# target_link_libraries (mls   ${PCL_LIBRARIES} )

add_executable (aligned_pcd2axis src/aligned_pcd2axis.cpp )
target_link_libraries (aligned_pcd2axis   ${PCL_LIBRARIES} )


add_executable (viewer_normal src/viewer_normal.cpp )
target_link_libraries (viewer_normal   ${PCL_LIBRARIES} )

link_directories( /home/liunao/qt/Note_to_ROS/tools/build )

# add_library(pack SHARED src/pack.cpp )
add_library(pack src/pack.cpp )
target_link_libraries(pack  ${PCL_LIBRARIES} )

SET(src_file main)
add_executable (${src_file} src/main.cpp )
target_link_libraries (${src_file} pack )
 
# SET(src_file generate_intensity_img)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}   ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES} )

# SET(src_file cal_eigen_value)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}   ${PCL_LIBRARIES} Ceres::ceres gflags)

SET(src_file passthrough)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} )


SET(src_file compute_normal)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} )


SET(src_file down_filtered_ply)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} )




SET(src_file bin_pcd)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   ${PCL_LIBRARIES} )
 

SET(src_file read_gicp_result)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  )

SET(src_file read_xml)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  )

SET(src_file four_param_tf)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  )


SET(src_file run_hba)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   )

# SET(src_file trans_gt_world)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# SET(src_file get_mean_pose_from_hdl)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


SET(src_file cal_camera2imu)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}   )


# SET(src_file get_box_point_for_cab)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${PCL_LIBRARIES}  )


# SET(src_file cal_gt_200)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${OpenCV_LIBS}  )

SET(src_file run_livo)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file} ${PCL_LIBRARIES} )


# SET(src_file run_livos)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  )


SET(src_file run_hdl)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


SET(src_file read_pose_from_jpu)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file} )

SET(src_file read_pose_from_loc)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file} )

SET(src_file rotation_pcd_to_horizon_pcl)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file} ${PCL_LIBRARIES} )


# SET(src_file remove_car)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# # SET(src_file extract_car)
# # add_executable (${src_file} src/${src_file}.cpp )
# # target_link_libraries (${src_file}  ${PCL_LIBRARIES} )

SET(src_file do_icp)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  ${PCL_LIBRARIES} )

SET(src_file normal_pcd)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# SET(src_file do_gicp_head_tail)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


