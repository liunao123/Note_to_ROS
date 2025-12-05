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
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include "pointcloud_utils.hpp"

namespace fs = std::filesystem;

// 创建clip目录结构并保存点云和pose信息
// clip_id: clip编号
// output_base_dir: 基础输出目录
// cloud: 要保存的点云
// pose_matrix: 位姿矩阵（4x4）
// timestamp: 时间戳
void saveClipData(int clip_id, const std::string& output_base_dir, 
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                  const Eigen::Matrix4d& pose_matrix,
                  double timestamp) {
    // 创建clip目录
    std::string clip_dir = output_base_dir + "/clip_" + std::to_string(clip_id);
    std::string lidar_dir = clip_dir + "/lidar";
    std::string pose_dir = clip_dir + "/pose";
    
    // 创建子目录
    fs::create_directories(lidar_dir);
    fs::create_directories(pose_dir);
    
    // 生成文件名（使用时间戳，转换为毫秒）
    long long timestamp_ms = static_cast<long long>(timestamp * 1000);
    std::string timestamp_str = std::to_string(timestamp_ms);
    
    // 保存点云到lidar目录
    std::string pcd_filename = lidar_dir + "/" + timestamp_str + ".pcd";
    pcl::io::savePCDFileASCII(pcd_filename, *cloud);
    
    // 保存pose到pose目录（JSON格式）
    std::string pose_filename = pose_dir + "/" + timestamp_str + ".json";
    std::ofstream pose_file(pose_filename);
    if (pose_file.is_open()) {
        pose_file << "{\n";
        pose_file << "  \"timestamp\": " << std::fixed << std::setprecision(3) << timestamp << ",\n";
        pose_file << "  \"pose_matrix\": [\n";
        for (int i = 0; i < 4; i++) {
            pose_file << "    [";
            for (int j = 0; j < 4; j++) {
                pose_file << std::fixed << std::setprecision(10) << pose_matrix(i, j);
                if (j < 3) pose_file << ", ";
            }
            pose_file << "]";
            if (i < 3) pose_file << ",";
            pose_file << "\n";
        }
        pose_file << "  ]\n";
        pose_file << "}\n";
        pose_file.close();
    }
    
    std::cout << "Saved clip " << clip_id << " to " << clip_dir << std::endl;
    std::cout << "  - Lidar: " << pcd_filename << " (" << cloud->size() << " points)" << std::endl;
    std::cout << "  - Pose: " << pose_filename << std::endl;
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
    std::cout << "Usage:  ---------get all pointclouds from one session-----------" << std::endl;
    std::cout << "Usage: " << argv[0] << " <pointclouds_dir> <poses_dir> <output_dir>" << std::endl;
    std::cout << "Usage:  --------------------------------------------------------" << std::endl;
    if (argc != 4) {
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

    Eigen::Vector3d first_pose(0.0, 0.0, 0.0);


    int cut = 0;
    for (const auto& pcd_file : pcd_files) {
        // if (cut++ % 10 != 0 )
        // if (cut++ > 3 )
        // {
        //     continue;
        // }
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

            if (first_pose == Eigen::Vector3d(0.0, 0.0, 0.0))
            {
                first_pose = pose_data.offset_utm;
            }

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
    // for (auto& point : map_cloud->points) {
    //     // 使用double精度进行减法运算，避免精度丢失
    //     point.x = static_cast<float>(static_cast<double>(point.x) + first_pose(0));
    //     point.y = static_cast<float>(static_cast<double>(point.y) + first_pose(1) );
    //     point.z = static_cast<float>(static_cast<double>(point.z) + first_pose(2));
    // }
    
    std::cout << "Translation completed. Points transformed to local coordinate system." << std::endl;

    // 保存变换后的点云
    std::string output_file = output_dir + "/map.pcd";
    if (pcl::io::savePCDFileBinary(output_file, *map_cloud) == -1)
    {
        std::cerr << "Could not save PCD file: " << output_file << std::endl;
        error_count++;
    }

    int grid_x_num = 4; // 默认X方向2个网格
    int grid_y_num = 4; // 默认Y方向2个网格

    if (grid_y_num  * grid_x_num == 1 )
    {
        return -1;
    }

    // 计算边界框
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*map_cloud, minPt, maxPt);
    
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
    for (const auto& point : map_cloud->points) {
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

            // 应用体素滤波，分辨率为0.25m
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(gridClouds[i]);
            const float voxel_size = 0.25f;
            voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);  // 0.25米分辨率
            voxel_filter.filter(*filtered_cloud);
            
            std::cout << "Grid " << i << ": Original points: " << gridClouds[i]->size() 
                      << ", After voxel filter: " << filtered_cloud->size() << std::endl;

            // 构造一个固定的pose矩阵（单位矩阵）
            Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
            // 可以根据需要修改pose值，例如使用first_pose
            // pose_matrix(0, 3) = first_pose(0);
            // pose_matrix(1, 3) = first_pose(1);
            // pose_matrix(2, 3) = first_pose(2);
            
            // 使用当前时间作为时间戳
            double current_timestamp = static_cast<double>(std::time(nullptr));
            
            // 使用新函数保存到clip结构中
            saveClipData(i + 1, output_dir, filtered_cloud, pose_matrix, current_timestamp);
            
            // 旧的保存方式（已注释）
            // std::string output_filename = output_dir +"/grid_" + std::to_string(i) + ".pcd";
            // pcl::io::savePCDFileASCII(output_filename, *filtered_cloud);
            // std::cout << "Saved grid " << i << " with " << filtered_cloud->size() << " points to " << output_filename << " (ASCII format)" << std::endl;
        }
    }

    std::cout << "Processing completed!" << std::endl;
    std::cout << "Successfully processed: " << processed_count << " files" << std::endl;
    std::cout << "files : " << output_file << std::endl;

    return 0;
}
