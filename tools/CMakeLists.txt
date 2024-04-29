cmake_minimum_required(VERSION 2.8)

project(rotation_pcd_to_horizon_pcl)

find_package(PCL  REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(   ${EIGEN3_INCLUDE_DIR}  )

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(src_file read_gicp_result)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  )

SET(src_file read_xml)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  )

# SET(src_file four_param_tf)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  )

# SET(src_file remove_outlier_pt)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}    ${PCL_LIBRARIES}   )


# SET(src_file run_hba)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}   )

# SET(src_file trans_gt_world)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )

# SET(src_file test)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}   )


# SET(src_file get_mean_pose_from_hdl)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# SET(src_file cal_camera2imu)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}   )


# SET(src_file get_box_point_for_cab)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${PCL_LIBRARIES}  )

# SET(src_file add_pcd)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${PCL_LIBRARIES}  )


# SET(src_file cal_gt_200)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${OpenCV_LIBS}  )

# SET(src_file run_livo)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${PCL_LIBRARIES} )


# SET(src_file run_livos)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  )


# SET(src_file run_hdl)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# SET(src_file read_pose_from_jpu)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} )


# SET(src_file rotation_pcd_to_horizon_pcl)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file} ${PCL_LIBRARIES} )


SET(src_file down_filtered_pcd)
add_executable (${src_file} src/${src_file}.cpp )
target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# SET(src_file remove_car)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# # SET(src_file extract_car)
# # add_executable (${src_file} src/${src_file}.cpp )
# # target_link_libraries (${src_file}  ${PCL_LIBRARIES} )

# SET(src_file do_icp)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )

# SET(src_file normal_pcd)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )


# SET(src_file do_gicp_head_tail)
# add_executable (${src_file} src/${src_file}.cpp )
# target_link_libraries (${src_file}  ${PCL_LIBRARIES} )



