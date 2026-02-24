#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <chrono>

// 遍历文件夹中的所有PCD文件
std::vector<std::string> getPcdFiles(const std::string& folder_path) {
    std::vector<std::string> pcd_files;
    try {
        // 检查文件夹是否存在
        if (!std::filesystem::exists(folder_path)) {
            std::cerr << "Error: Directory does not exist: " << folder_path << std::endl;
            return pcd_files;
        }
        if (!std::filesystem::is_directory(folder_path)) {
            std::cerr << "Error: Path is not a directory: " << folder_path << std::endl;
            return pcd_files;
        }
        // 遍历目录
        for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
            if (entry.is_regular_file()) {
                std::string file_path = entry.path().string();
                std::string file_extension = entry.path().extension().string();
                
                // 检查是否为PCD文件（不区分大小写）
                std::transform(file_extension.begin(), file_extension.end(), 
                             file_extension.begin(), ::tolower);
                
                if (file_extension == ".pcd") {
                    pcd_files.push_back(file_path);
                }
            }
        }
        // 按文件名排序
        std::sort(pcd_files.begin(), pcd_files.end());
        std::cout << "Found " << pcd_files.size() << " PCD files in: " << folder_path << std::endl;
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "Filesystem error: " << ex.what() << std::endl;
    }
    return pcd_files;
}

struct PoseData {
    double timestamp;
    Eigen::Affine3d transformation_matrix;
    Eigen::Vector3d offset_utm;
};
Eigen::Vector3d first_pose_at_utm;

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
        long double offset_x = offset_utm[0].as<long double>();
        long double offset_y = offset_utm[1].as<long double>();
        long double offset_z = offset_utm[2].as<long double>();
        pose_data.offset_utm(0) = static_cast<double>(offset_x);
        pose_data.offset_utm(1) = static_cast<double>(offset_y);
        pose_data.offset_utm(2) = static_cast<double>(offset_z);
        first_pose_at_utm = pose_data.offset_utm;
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file " << yaml_file << ": " << e.what() << std::endl;
        throw;
    }
    return pose_data;
}

int main(int argc, char** argv) 
{
    std::string filename = "/mnt/nvme0n1p2/data/1011/dense_global_map_xyzi.pcd";
    int grid_x_num = 3; // 默认X方向2个网格
    int grid_y_num = 3; // 默认Y方向2个网格

    if (argc >= 2) {
        filename = argv[1];
    }
    if (argc >= 3) {
        grid_x_num = std::stoi(argv[2]);
    }
    if (argc >= 4) {
        grid_y_num = std::stoi(argv[3]);
    }

    auto result = readPoseFromYaml(filename + "/sparse/vehicle_geo_pose/0_1763964284.000.yaml");
    std::cout << "first_pose_at_utm: " << first_pose_at_utm.transpose() << std::endl << std::endl;

    // 提取最后一个/之前的所有字符（即目录路径）
    std::string dir_path = "";
    size_t last_slash = filename.find_last_of('/');
    if (last_slash != std::string::npos) {
        dir_path = filename.substr(0, last_slash);
    }

    std::cout << "filename: " << filename << std::endl;
    std::cout << "directory path: " << dir_path << std::endl;
    std::cout << "Grid size: " << grid_x_num << "x" << grid_y_num << " = " << grid_x_num * grid_y_num << " grids" << std::endl;
    
    // mc cp /mnt/nvme0n1p2/data/0930/build/hesai_0930/ minio-rsu/tyjt-rsu/qcsl_map/map   --recursive

    Eigen::Affine3d T_wl = Eigen::Affine3d::Identity();
    Eigen::Vector3d project_original_utm( 662276.0 , 4873428.0, 200.0 );

    T_wl.translation() =  first_pose_at_utm - first_pose_at_utm;
    // T_wl.translation() =  first_pose_at_utm - project_original_utm;

    Eigen::Quaterniond q1( 1.0, 0.0, 0.0, 0.0  );
    T_wl.rotate(q1);
    std::cout << "T_wl: " << T_wl.matrix()  << std::endl << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>);

    // 示例：遍历目录中的所有PCD文件
    if (!dir_path.empty())
    {
        std::vector<std::string> pcd_files = getPcdFiles(dir_path);
        std::cout << pcd_files.size() << " PCD files found:" << std::endl;
        // for (const auto &file : pcd_files)
        for (size_t i = 0; i < pcd_files.size(); i=i+1)
        {
            const auto &file = pcd_files[i];
            std::cout << "load pcd:  " << file << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>);
            // 获取文件大小信息
            auto file_size = std::filesystem::file_size(file);
            std::cout << "File size: " << file_size / (1024 * 1024) << " MB" << std::endl;
            
            // 尝试加载PCD文件，使用file变量而不是filename
            std::cout << "Loading compressed PCD file..." << std::endl;
            auto start_time = std::chrono::high_resolution_clock::now();
            
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *cloud_o) == -1)
                {
                    std::cerr << "Error: Couldn't read file: " << file << std::endl;
                    continue;  // 跳过这个文件，继续处理下一个
                }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            std::cout << "Loading completed in " << duration.count() << " ms" << std::endl;
            std::cout << "cloud_o->size(): " << cloud_o->size() << std::endl;

            // 对点云进行变换
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud_o, *transformedCloud, T_wl);
            *global_map += *transformedCloud;
            std::cout << "global_map->size(): " << global_map->size() << std::endl;
        }
    }
    
    std::string output_filename = dir_path + "/utm.pcd";
    pcl::io::savePCDFileBinary(output_filename, *global_map);
    std::cout << "global_map->size(): " << global_map->size() << std::endl;
    std::cout << "output_filename: " << output_filename << std::endl;

    // return -1;

    if (grid_y_num  * grid_x_num == 1 )
    {
        return -1;
    }

    // 计算边界框
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*global_map, minPt, maxPt);
    
    float x_range = maxPt.x - minPt.x;
    float y_range = maxPt.y - minPt.y;
    float grid_x_size = x_range / grid_x_num;
    float grid_y_size = y_range / grid_y_num;
    
    int total_grids = grid_x_num * grid_y_num;
    std::cout << "X range: " << x_range << ", Y range: " << y_range << std::endl;
    std::cout << "Grid size: " << grid_x_size << " x " << grid_y_size << std::endl;

    // 创建网格点云容器
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> gridClouds(total_grids);
    for (int i = 0; i < total_grids; i++) {
        gridClouds[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    // 将点分配到网格
    for (const auto& point : global_map->points) {
        int grid_x_idx = std::min(grid_x_num - 1, (int)((point.x - minPt.x) / grid_x_size));
        int grid_y_idx = std::min(grid_y_num - 1, (int)((point.y - minPt.y) / grid_y_size));
        int grid_idx = grid_y_idx * grid_x_num + grid_x_idx;
        
        if (grid_idx >= 0 && grid_idx < total_grids) {
            gridClouds[grid_idx]->points.push_back(point);
        }
    }

    // 保存网格文件
    for (int i = 0; i < total_grids; i++) {
        if (!gridClouds[i]->points.empty()) {
            gridClouds[i]->width = gridClouds[i]->size();
            gridClouds[i]->height = 1;
            output_filename = dir_path +"/grid_" + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(output_filename, *gridClouds[i]);
            std::cout << "Saved grid " << i << " with " << gridClouds[i]->size() << " points to " << output_filename << std::endl;
        }
    }

    return 0;
}
