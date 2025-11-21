#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include <pcl/common/common.h>
#include <algorithm>

namespace fs = std::filesystem;

Eigen::Vector3d first_pose;


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
        long double offset_x = offset_utm[0].as<long double>();
        long double offset_y = offset_utm[1].as<long double>();
        long double offset_z = offset_utm[2].as<long double>();
        
        pose_data.offset_utm(0) = static_cast<double>(offset_x);
        pose_data.offset_utm(1) = static_cast<double>(offset_y);
        pose_data.offset_utm(2) = static_cast<double>(offset_z);
        first_pose = pose_data.offset_utm;
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file " << yaml_file << ": " << e.what() << std::endl;
        throw;
    }
    
    return pose_data;
}

// 从文件名提取时间戳字符串
std::string extractTimestamp(const std::string& filename) {
    size_t pos = filename.find('_');
    if (pos != std::string::npos) {
        size_t end_pos = filename.find('.', pos);
        if (end_pos != std::string::npos) {
            return filename.substr(pos + 1, end_pos - pos - 1);
        }
    }
    return "";
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Usage: " << argv[0] << " <pointclouds_dir> <poses_dir> <output_dir>" << std::endl;
        return -1;
    }
    
    std::string pointclouds_dir = argv[1];
    std::string poses_dir = argv[2];
    std::string output_dir = argv[3];
    
    // 创建输出目录
    fs::create_directories(output_dir);
    
    // 获取所有PCD文件
    std::vector<std::string> pcd_files;
    for (const auto& entry : fs::directory_iterator(pointclouds_dir)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path().filename().string());
        }
    }
    
    // 排序文件以确保顺序处理
    std::sort(pcd_files.begin(), pcd_files.end());
    
    std::cout << "Found " << pcd_files.size() << " PCD files" << std::endl;
    
    int processed_count = 0;
    int error_count = 0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int cut = 0;
    for (const auto& pcd_file : pcd_files) {
        // if (cut++ % 10 != 0 )
        if (cut++ > 3 )
        {
            continue;
        }
        try {
            // 提取时间戳
            std::string timestamp_str = extractTimestamp(pcd_file);
            if (timestamp_str.empty()) {
                std::cerr << "Could not extract timestamp from " << pcd_file << std::endl;
                error_count++;
                continue;
            }
            
            // 构建对应的YAML文件名
            std::string yaml_file = poses_dir + "/" + pcd_file.substr(0, pcd_file.length() - 4) + ".yaml";
            // std::cerr << "yaml_file file: " << yaml_file << std::endl;
            // std::cerr << "pcd_file file: " << pcd_file << std::endl;
            
            // 检查YAML文件是否存在
            if (!fs::exists(yaml_file)) {
                std::cerr << "Pose file not found: " << yaml_file << std::endl;
                error_count++;
                continue;
            }
            
            // 读取点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>);
            std::string pcd_path = pointclouds_dir + "/" + pcd_file;
            
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud_o) == -1) {
                std::cerr << "Could not read PCD file: " << pcd_path << std::endl;
                error_count++;
                continue;
            }
                // std::cout << "cloud_o " << cloud_o->size()  << std::endl;
            
            // 过滤距离原点小于3米的点
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& point : cloud_o->points) {
                double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if ( point.z < -4.0 || point.z > 20.0 )
                {
                    continue;
                }
                
                if (distance >= 3.0f) {  // 保留距离大于等于3米的点
                    cloud->points.push_back(point);
                }
            }
            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = true;
            // std::cout << "cloud " << cloud->size()  << std::endl;
            
            // 使用过滤后的点云
            // cloud = filtered_cloud;
            
            // 读取位姿数据
            PoseData pose_data = readPoseFromYaml(yaml_file);
            // std::cerr << "pose_data  : " << pose_data.timestamp << std::endl;
           
            // 创建完整的变换矩阵：先加偏移量到变换矩阵的平移部分
            Eigen::Affine3d full_transform = pose_data.transformation_matrix;

            // 应用变换
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            
            // 方法1: 使用完整的变换矩阵（推荐）
            pcl::transformPointCloud(*cloud, *transformed_cloud, full_transform);
 
            *map_cloud  += *transformed_cloud;
            
            processed_count++;
            if (processed_count % 100 == 0) {
                std::cout << "Processed " << processed_count << " files..." << std::endl;
                std::cout << "map_cloud " << map_cloud->size()  << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing " << pcd_file << ": " << e.what() << std::endl;
            error_count++;
        }
    }
    
    std::cout << "all map_cloud " << map_cloud->size()   << std::endl;
    std::cout << "first_pose " << std::fixed << std::setprecision(6) << first_pose.transpose() << std::endl;

    // 为了保证精度，手动遍历每个点进行高精度变换，而不是使用Affine3d
    // 因为大数值的UTM坐标在变换时容易丢失精度
    std::cout << "Applying high-precision translation offset..." << std::endl;
    
    // 平移分量太大，在cc里面查看 是不正常的
    for (auto& point : map_cloud->points) {
        // 使用double精度进行减法运算，避免精度丢失
        point.x = static_cast<float>(static_cast<double>(point.x) + first_pose(0));
        point.y = static_cast<float>(static_cast<double>(point.y) + first_pose(1) );
        point.z = static_cast<float>(static_cast<double>(point.z) + first_pose(2));
    }
    
    std::cout << "Translation completed. Points transformed to local coordinate system." << std::endl;

    // 保存变换后的点云
    std::string output_file = "/mnt/nvme0n1p2/data/2.pcd";
    if (pcl::io::savePCDFileBinary(output_file, *map_cloud) == -1)
    {
        std::cerr << "Could not save PCD file: " << output_file << std::endl;
        error_count++;
    }

    std::cout << "Processing completed!" << std::endl;
    std::cout << "Successfully processed: " << processed_count << " files" << std::endl;
    std::cout << "Errors: " << error_count << " files" << std::endl;
    std::cout << "files : " << output_file << std::endl;

    return 0;
}
