cmake_minimum_required(VERSION 3.16)
project(dwm-mapping-postprocessing)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add compile flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
# Workaround for FLANN serialization issue with newer GCC  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(libLAS REQUIRED)

# 查找jsoncpp
pkg_check_modules(JSONCPP jsoncpp)

# 查找RapidJSON
find_package(RapidJSON QUIET)

if(NOT RapidJSON_FOUND)
    message(STATUS "RapidJSON not found via find_package, using system include")
    set(RAPIDJSON_INCLUDE_DIRS /usr/include/rapidjson)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tydwm
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${PROJECT_SOURCE_DIR}/thirdparty/rapidjson
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

# 设置源代码目录
set(OBJ_DIR ${CMAKE_SOURCE_DIR}/include/obj)

# 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${RAPIDJSON_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# 收集源文件
set(OBJ_SOURCES
    ${OBJ_DIR}/ObjectAnnotation.cpp
    ${OBJ_DIR}/ObjectAnnotationLoader.cpp
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})


# Non-uniform sampling standalone program
add_executable(06_nu_sampling
    src/06_nu_sampling.cc
    src/NonUniformSampling.cc
    src/sensor_model.cc
    src/OctreeResample.cc
    src/tydwm/HdMapNew.cpp
    ${OBJ_SOURCES}
)

target_link_libraries(06_nu_sampling
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Eigen3::Eigen
    glog::glog
    OpenMP::OpenMP_CXX
)

set_target_properties(06_nu_sampling
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build
)

# Optional: Print some information
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")

# CPack configuration for DEB package
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "dwm-mp")
set(CPACK_PACKAGE_VERSION "1.1.0")
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "your-email@example.com")
set(CPACK_PACKAGE_VENDOR "Your Organization")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "DWM Mapping Postprocessing - Point Cloud Mapping and Processing Tool")
set(CPACK_PACKAGE_DESCRIPTION "A comprehensive point cloud mapping and processing tool
that supports LiDAR merge, RGBD fusion, MVSLAM, road segmentation,
and depth estimation for autonomous driving applications.")

# DEB specific settings
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Your Name <your-email@example.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")

# Automatically detect dependencies
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

# Manual dependency specification (fallback if auto-detection doesn't work)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libopencv-dev (>= 4.5.0), libpcl-dev (>= 1.10.0), libeigen3-dev (>= 3.3.0), libceres-dev (>= 2.0.0), libgoogle-glog-dev, libyaml-cpp-dev, libglew-dev")

# Install changelog file if exists
if(EXISTS "${CMAKE_SOURCE_DIR}/debian/changelog")
    install(FILES "${CMAKE_SOURCE_DIR}/debian/changelog"
            DESTINATION "share/doc/${CPACK_PACKAGE_NAME}"
            RENAME changelog.Debian)
endif()

# Package file name
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

# Include CPack
include(CPack)
