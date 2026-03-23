#pragma once

#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <liblas/liblas.hpp>
#include <fstream>
#include <filesystem>
#include <ctime>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "file_utils.hpp"

bool init_flag = false;
double OFFSET_X = 0.0;
double OFFSET_Y = 0.0;
double OFFSET_Z = 0.0;



struct PoseData
{
    double timestamp;
    Eigen::Affine3d transformation_matrix;
    Eigen::Vector3d offset_utm;
};

inline PoseData readPoseFromYaml(const std::string &yaml_file)
{
    PoseData pose_data;
    pose_data.transformation_matrix = Eigen::Affine3d::Identity();

    try
    {
        YAML::Node config = YAML::LoadFile(yaml_file);

        // 读取时间戳 - 使用高精度
        pose_data.timestamp = config["timestamp"].as<long double>();

        // 读取变换矩阵 (4x4) - 使用高精度
        auto pose_utm = config["pose_utm"];
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

        for (int i = 0; i < 16; ++i)
        {
            int row = i / 4;
            int col = i % 4;
            // 使用long double确保精度，然后转换为double
            long double value = pose_utm[i].as<long double>();
            matrix(row, col) = static_cast<double>(value);
            // std::cout << std::fixed << std::setprecision(15) << "pose_utm[" << i << "]: " << value << std::endl;
        }

        // 将Matrix4d转换为Affine3d
        pose_data.transformation_matrix = Eigen::Affine3d(matrix);

        // 读取偏移量 - 使用高精度
        auto offset_utm = config["offset_utm"];
        pose_data.offset_utm(0) = offset_utm[0].as<long double>();
        pose_data.offset_utm(1) = offset_utm[1].as<long double>();
        pose_data.offset_utm(2) = offset_utm[2].as<long double>();

        if (!init_flag)
        {
            init_flag = true;
            OFFSET_X = pose_data.offset_utm(0);
            OFFSET_Y = pose_data.offset_utm(1);
            OFFSET_Z = pose_data.offset_utm(2);
            // std::cout << std::fixed << std::setprecision(3) << "OFFSET_X: " << OFFSET_X << std::endl;
            // std::cout << std::fixed << std::setprecision(3) << "OFFSET_Y: " << OFFSET_Y << std::endl;
            // std::cout << std::fixed << std::setprecision(3) << "OFFSET_Z: " << OFFSET_Z << std::endl;
        }
        pose_data.offset_utm = pose_data.offset_utm ; // - Eigen::Vector3d(OFFSET_X, OFFSET_Y, OFFSET_Z);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading YAML file " << yaml_file << ": " << e.what() << std::endl;
        throw;
    }

    return pose_data;
}

inline bool writePoseToYaml(const PoseData &pose, const std::string &yaml_file)
{
    try
    {
        YAML::Node root;
        const double timestamp_3dp = std::round(pose.timestamp * 1000.0) / 1000.0;
        std::ostringstream ts_stream;
        ts_stream << std::fixed << std::setprecision(3) << timestamp_3dp;
        root["timestamp"] = ts_stream.str();

        Eigen::Matrix4d matrix = pose.transformation_matrix.matrix();
        YAML::Node pose_utm;
        for (int i = 0; i < 16; ++i)
        {
            int row = i / 4;
            int col = i % 4;
            pose_utm.push_back(static_cast<long double>(matrix(row, col)));
        }
        root["pose_utm"] = pose_utm;

        YAML::Node offset_utm;
        offset_utm.push_back(static_cast<long double>(pose.offset_utm(0)));
        offset_utm.push_back(static_cast<long double>(pose.offset_utm(1)));
        offset_utm.push_back(static_cast<long double>(pose.offset_utm(2)));
        root["offset_utm"] = offset_utm;

        std::ofstream fout(yaml_file);
        if (!fout)
        {
            std::cerr << "Error opening YAML file for writing: " << yaml_file << std::endl;
            return false;
        }

        fout << root;
        fout.close();
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error writing YAML file " << yaml_file << ": " << e.what() << std::endl;
        return false;
    }
}



inline void writeOffsetFile(double offset_x, double offset_y, double offset_z, const std::string& file_path)
{
    YAML::Node root;
    root["offset_x"] = offset_x;
    root["offset_y"] = offset_y;
    root["offset_z"] = offset_z;

    std::ofstream fout(file_path);
    if (!fout) {
        std::cerr << "Error opening file for writing: " << file_path << std::endl;
        return;
    }
    fout << root;
    fout.close();
}


template <typename PointT>
inline void writePly(const pcl::PointCloud<PointT> &cloud, const std::string &output_ply)
{
    if (pcl::io::savePLYFile(output_ply, cloud) == -1)
    {
        std::cerr << "Could not save PLY file: " << output_ply << std::endl;
    }
}

template <typename PointT>
inline void writePcd(const pcl::PointCloud<PointT> &cloud, const std::string &output_pcd)
{
    if (pcl::io::savePCDFile(output_pcd, cloud) == -1)
    {
        std::cerr << "Could not save PCD file: " << output_pcd << std::endl;
    }
}


inline pcl::PointCloud<pcl::PointXYZI>::Ptr loadCorrespondingPointCloud(const std::string &pose_file_path,
                                                                 const std::vector<std::string> &all_pcd_files)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::string pcd_file_path = findCorrespondingPcdFile(pose_file_path, all_pcd_files);
    if (pcd_file_path.empty())
    {
        return cloud;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path, *cloud) == -1)
    {
        std::cerr << "Could not read PCD file: " << pcd_file_path << std::endl;
        cloud->clear();
        return cloud;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &point : cloud->points)
    {
        if (std::fabs(point.x) < 2.0 && std::fabs(point.y) < 2.0)
        {
            continue;
        }
        filtered_cloud->points.push_back(point);
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    return filtered_cloud;
}

