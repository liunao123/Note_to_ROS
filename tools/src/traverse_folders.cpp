#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <algorithm>


namespace fs = std::filesystem;

// 递归遍历两层目录结构，返回指定后缀的文件路径
// extension: 文件后缀，如".pcd", ".txt", ".yaml"等。如果为空字符串则返回所有文件
std::vector<std::string> getAllFilePaths(const std::string& root_dir, const std::string& extension = "") {
    std::vector<std::string> file_paths;
    
    if (!fs::exists(root_dir) || !fs::is_directory(root_dir)) {
        std::cerr << "Error: Directory does not exist: " << root_dir << std::endl;
        return file_paths;  // 返回空vector
    }
    
    try {
        // 遍历第一层目录
        for (const auto& level1_entry : fs::directory_iterator(root_dir)) {
            if (level1_entry.is_directory()) {
                std::string level1_path = level1_entry.path().string();
                
                // 遍历第二层目录
                for (const auto& level2_entry : fs::directory_iterator(level1_path)) {
                    if (level2_entry.is_directory()) {
                        std::string level2_path = level2_entry.path().string();
                        
                        // 遍历第二层目录中的所有文件
                        for (const auto& file_entry : fs::directory_iterator(level2_path)) {
                            if (file_entry.is_regular_file()) {
                                // 检查文件后缀
                                if (extension.empty() || file_entry.path().extension() == extension) {
                                    file_paths.push_back(file_entry.path().string());
                                }
                            }
                        }
                    } else if (level2_entry.is_regular_file()) {
                        // 如果第一层目录直接包含文件
                        if (extension.empty() || level2_entry.path().extension() == extension) {
                            file_paths.push_back(level2_entry.path().string());
                        }
                    }
                }
            }
        }
    } catch (const fs::filesystem_error& ex) {
        std::cerr << "Filesystem error: " << ex.what() << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
    
    return file_paths;
}

struct PoseData {
    double timestamp;
    Eigen::Affine3d transformation_matrix;
    Eigen::Vector3d offset_utm;
};

// 从YAML文件读取位姿数据
PoseData readPoseFromYaml(const std::string& yaml_file) {
    PoseData pose_data;
    pose_data.transformation_matrix = Eigen::Affine3d::Identity();
    
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        // 读取时间戳 - 使用高精度
        pose_data.timestamp = config["timestamp"].as<long double>();
        
        // 读取变换矩阵 (4x4) - 使用高精度
        auto pose_utm = config["pose_utm"];
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        
        for (int i = 0; i < 16; ++i) {
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

    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file " << yaml_file << ": " << e.what() << std::endl;
        throw;
    }
    
    return pose_data;
}

// 在指定半径内搜索位置的函数，并保存搜索结果到PCD文件
std::vector<int> searchNearbyPoses(pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud,
                                   const pcl::PointXYZI& query_point,
                                   float search_radius,
                                   const std::string& output_filename = "") {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    if (kdtree->radiusSearch(query_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        std::cout << "Found " << pointIdxRadiusSearch.size() << " poses within " << search_radius << "m radius" << std::endl;
        
        // 创建新的点云来存储搜索结果
        pcl::PointCloud<pcl::PointXYZI>::Ptr search_result_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            int pose_idx = pointIdxRadiusSearch[i];
            std::cout << "  Pose " << pose_idx << " (distance: " 
                      << sqrt(pointRadiusSquaredDistance[i]) << "m)" << std::endl;
            
            // 将搜索到的点添加到结果点云中
            search_result_cloud->points.push_back(original_cloud->points[pose_idx]);
        }
        
        // 设置点云属性
        search_result_cloud->width = search_result_cloud->points.size();
        search_result_cloud->height = 1;
        search_result_cloud->is_dense = true;
        
        // 保存搜索结果到PCD文件
        if (!output_filename.empty()) {
            if (pcl::io::savePCDFile(output_filename, *search_result_cloud) == -1) {
                std::cerr << "Could not save search result PCD file: " << output_filename << std::endl;
            } else {
                std::cout << "Search result saved to: " << output_filename << std::endl;
            }
        }
    } else {
        std::cout << "No poses found within " << search_radius << "m radius" << std::endl;
    }
    
    return pointIdxRadiusSearch;
}

// K近邻搜索函数，并保存搜索结果到PCD文件
std::vector<int> searchKNearestPoses(pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud,
                                     const std::vector<PoseData>& pose_data_vector,
                                     const pcl::PointXYZI& query_point,
                                     int k,
                                     const std::string& output_filename = "") {
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);
    
    if (kdtree->nearestKSearch(query_point, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        std::cout << "Found " << k << " nearest poses:" << std::endl;
        
        // 创建新的点云来存储搜索结果
        pcl::PointCloud<pcl::PointXYZI>::Ptr search_result_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
            int pose_idx = pointIdxNKNSearch[i];
            std::cout << "  Pose " << pose_idx << " (distance: " 
                      << sqrt(pointNKNSquaredDistance[i]) << "m, timestamp: " 
                      << pose_data_vector[pose_idx].timestamp << ")" << std::endl;
            
            // 将搜索到的点添加到结果点云中
            search_result_cloud->points.push_back(original_cloud->points[pose_idx]);
        }
        
        // 设置点云属性
        search_result_cloud->width = search_result_cloud->points.size();
        search_result_cloud->height = 1;
        search_result_cloud->is_dense = true;
        
        // 保存搜索结果到PCD文件
        if (!output_filename.empty()) {
            if (pcl::io::savePCDFile(output_filename, *search_result_cloud) == -1) {
                std::cerr << "Could not save search result PCD file: " << output_filename << std::endl;
            } else {
                std::cout << "Search result saved to: " << output_filename << std::endl;
            }
        }
    }
    
    return pointIdxNKNSearch;
}

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cout << "Usage: " << argv[0] << " <root_directory> [file_extension]" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/your/folders" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/your/folders .pcd" << std::endl;
        std::cout << "Example: " << argv[0] << " /path/to/your/folders .yaml" << std::endl;
        return -1;
    }
    
    std::string root_dir = argv[1];
    std::string extension = (argc == 3) ? argv[2] : "";
    
    std::cout << "Traversing directory: " << root_dir << std::endl;
    if (!extension.empty()) {
        std::cout << "Looking for files with extension: " << extension << std::endl;
    } else {
        std::cout << "Looking for all files" << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "get all key frame  pose ...... " << std::endl;
    
    // 调用函数获取指定后缀的文件路径
    std::vector<std::string> all_pose_files = getAllFilePaths(root_dir, extension);
    
    // 创建点云来存储位姿的translation部分
    pcl::PointCloud<pcl::PointXYZI>::Ptr pose_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // 创建vector来存储所有的pose_data，与点云索引对应
    std::vector<PoseData> all_pose_data;
    
    std::cout << "get all key frame  pose done...... " << std::endl;
    
    // 输出所有文件路径并处理每个位姿
    for (const auto& file_path : all_pose_files) {
        // std::cout << "File: " << file_path << std::endl; 
        try {
            PoseData pose_data = readPoseFromYaml(file_path);
            // std::cout << "pose_data: " << pose_data.timestamp << std::endl; 
            // std::cout << "pose_data: " << pose_data.offset_utm << std::endl; 
            
            // 获取translation向量
            Eigen::Vector3d translation = pose_data.transformation_matrix.translation() + pose_data.offset_utm;
            // std::cout << "transformation_matrix: " << translation.transpose() << std::endl;
            
            // 将translation添加到点云中
            pcl::PointXYZI point;
            point.x = static_cast<float>(translation.x());
            point.y = static_cast<float>(translation.y());
            point.z = static_cast<float>(translation.z());
            point.intensity = static_cast<float>(pose_data.timestamp);  // 使用timestamp作为intensity
            
            pose_cloud->points.push_back(point);
            
            // 将pose_data添加到vector中，确保索引与点云对应
            all_pose_data.push_back(pose_data);
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing " << file_path << ": " << e.what() << std::endl;
        }
    }
    
    // 设置点云属性
    pose_cloud->width = pose_cloud->points.size();
    pose_cloud->height = 1;
    pose_cloud->is_dense = true;
    
    // 保存点云到文件
    std::string output_file = "/tmp/mapping_pose_utm.pcd";
    if (pcl::io::savePCDFile(output_file, *pose_cloud) == -1) {
        std::cerr << "Could not save PCD file: " << output_file << std::endl;
    } else {
        std::cout << "Pose translations saved to: " << output_file << std::endl;
    }

    std::cout << "start search knn ...... " << std::endl;
    // 构建KdTree用于快速搜索
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree->setInputCloud(pose_cloud);
    
    std::cout << "KdTree built successfully with " << pose_cloud->points.size() << " poses" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    // 演示搜索功能
    if (!pose_cloud->points.empty()) {
        // 使用第一个位姿作为查询点
        pcl::PointXYZI query_point = pose_cloud->points[100];
        std::cout << "Demo search using first pose as query point:" << std::endl;
        std::cout << "Query point: (" << query_point.x << ", " << query_point.y << ", " << query_point.z << ")" << std::endl;
        
        // 半径搜索示例 - 搜索100米范围内的位姿
        std::cout << "\n=== Radius Search (100m) ===" << std::endl;
        std::vector<int> found_indices = searchNearbyPoses(kdtree, pose_cloud, query_point, 100.0f, "/tmp/search_result_radius_100m.pcd");
        
        // 使用返回的索引访问对应的pose_data
        std::cout << "Found pose indices: ";
        for (int idx : found_indices) {
            std::cout << idx << " " << std::fixed << std::setprecision(3) << all_pose_data[idx].timestamp << " " << all_pose_data[idx].offset_utm.transpose() << std::endl;
        }
        std::cout << std::endl;

        // ! 构造出来pcd的文件名，再转换一下就ok了 20251121 by ln


        
        
        // K近邻搜索示例 - 搜索最近的5个位姿
        // std::cout << "\n=== K-Nearest Search (k=5) ===" << std::endl;
        // searchKNearestPoses(kdtree, pose_cloud, all_pose_data, query_point, 5, "/tmp/search_result_k5_nearest.pcd");
 
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Total files found: " << all_pose_files.size() << std::endl;
    std::cout << "Total poses saved: " << pose_cloud->points.size() << std::endl;
    std::cout << "Total pose_data stored: " << all_pose_data.size() << std::endl;
    
    return 0;
}
