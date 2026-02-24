#include <iostream>
#include <fstream>
#include <iomanip>
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
#include <Eigen/Geometry>

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
// images_base_dir: 图像文件的基础目录（可选）
void saveClipData(int clip_id, const std::string& output_base_dir, 
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                  const Eigen::Matrix4d& pose_matrix,
                  double timestamp,
                  const std::string& images_base_dir = "") 
    {
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
    
    // 从4x4矩阵提取位置和旋转
    Eigen::Vector3d translation = pose_matrix.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = pose_matrix.block<3, 3>(0, 0);
    
    // 转换为四元数
    Eigen::Quaterniond quaternion(rotation);
    
    // 转换为欧拉角（roll, pitch, yaw）
    Eigen::Vector3d euler = rotation.eulerAngles(0, 1, 2); // XYZ顺序
    
    std::ofstream pose_file(pose_filename);
    if (pose_file.is_open()) {
        pose_file << std::fixed << std::setprecision(10);
        pose_file << "{\n";
        pose_file << "  \"cname\": \"" << timestamp_str << ".pcd\",\n";
        pose_file << "  \"pose\": {\n";
        pose_file << "    \"x\": " << translation.x() << ",\n";
        pose_file << "    \"y\": " << translation.y() << ",\n";
        pose_file << "    \"z\": " << translation.z() << ",\n";
        pose_file << "    \"rx\": " << euler.x() << ",\n";
        pose_file << "    \"ry\": " << euler.y() << ",\n";
        pose_file << "    \"rz\": " << euler.z() << ",\n";
        pose_file << "    \"qw\": " << quaternion.w() << ",\n";
        pose_file << "    \"qx\": " << quaternion.x() << ",\n";
        pose_file << "    \"qy\": " << quaternion.y() << ",\n";
        pose_file << "    \"qz\": " << quaternion.z() << "\n";
        pose_file << "  }\n";
        pose_file << "}\n";
        pose_file.close();
    }
    
    std::cout << "Saved clip " << clip_id << " to " << clip_dir << std::endl;
    std::cout << "  - Lidar: " << pcd_filename << " (" << cloud->size() << " points)" << std::endl;
    std::cout << "  - Pose: " << pose_filename << std::endl;
    
    // 如果提供了图像目录，复制对应的图像
//     if (!images_base_dir.empty()) {
//         copyImagesByTimestamp(images_base_dir, timestamp, clip_dir);
//     }
}

// 根据时间戳查找并复制对应的图像文件
// images_base_dir: 图像文件的基础目录
// timestamp: 时间戳（秒）
// clip_dir: clip目录路径
// tolerance_ms: 时间戳容差（毫秒），默认50ms
void copyImagesByTimestamp(const std::string& images_base_dir, 
                          double timestamp,
                          const std::string& clip_dir,
                          int tolerance_ms = 50) {
    if (!fs::exists(images_base_dir)) {
        std::cerr << "Warning: Images directory not found: " << images_base_dir << std::endl;
        return;
    }
    
    // 创建clip的images目录
    std::string images_output_dir = clip_dir + "/images";
    fs::create_directories(images_output_dir);
    
    // 将时间戳转换为毫秒
    long long target_timestamp_ms = static_cast<long long>(timestamp * 1000);
    std::string timestamp_str = std::to_string(target_timestamp_ms);
    
    std::cout << "  Searching images for timestamp: " << timestamp_str << " ms" << std::endl;
    
    int copied_count = 0;
    
    // 遍历images目录下的所有子文件夹
    try {
        for (const auto& camera_dir : fs::directory_iterator(images_base_dir)) {
            if (!camera_dir.is_directory()) {
                continue;
            }
            
            std::string camera_name = camera_dir.path().filename().string();
            
            // 遍历相机文件夹中的所有图像文件
            for (const auto& image_entry : fs::directory_iterator(camera_dir.path())) {
                if (!image_entry.is_regular_file()) {
                    continue;
                }
                
                std::string image_filename = image_entry.path().filename().string();
                std::string extension = image_entry.path().extension().string();
                
                // 检查是否为图像文件
                if (extension != ".jpg" && extension != ".png" && 
                    extension != ".jpeg" && extension != ".bmp") {
                    continue;
                }
                
                // 从文件名提取时间戳
                // 假设文件名格式为: cam_1234567890123.jpg 或 1234567890123.jpg
                std::string img_timestamp_str;
                size_t underscore_pos = image_filename.find('_');
                if (underscore_pos != std::string::npos) {
                    // 格式: cam_1234567890123.jpg
                    size_t dot_pos = image_filename.find('.', underscore_pos);
                    if (dot_pos != std::string::npos) {
                        img_timestamp_str = image_filename.substr(underscore_pos + 1, 
                                                                 dot_pos - underscore_pos - 1);
                    }
                } else {
                    // 格式: 1234567890123.jpg
                    size_t dot_pos = image_filename.find('.');
                    if (dot_pos != std::string::npos) {
                        img_timestamp_str = image_filename.substr(0, dot_pos);
                    }
                }
                
                if (img_timestamp_str.empty()) {
                    continue;
                }
                
                try {
                    long long img_timestamp_ms = std::stoll(img_timestamp_str);
                    long long time_diff = std::abs(img_timestamp_ms - target_timestamp_ms);
                    
                    // 如果时间差在容差范围内，复制该图像
                    if (time_diff <= tolerance_ms) {
                        // 构建新的文件名：timestamp_cameraname.extension
                        std::string new_filename = timestamp_str + "_" + camera_name + extension;
                        std::string dest_path = images_output_dir + "/" + new_filename;
                        
                        // 复制文件
                        fs::copy_file(image_entry.path(), dest_path, 
                                     fs::copy_options::overwrite_existing);
                        
                        copied_count++;
                        std::cout << "    Copied image: " << camera_name << "/" << image_filename 
                                  << " -> " << new_filename 
                                  << " (time diff: " << time_diff << " ms)" << std::endl;
                    }
                } catch (const std::exception& e) {
                    // 时间戳解析失败，跳过该文件
                    continue;
                }
            }
        }
        
        if (copied_count > 0) {
            std::cout << "  - Images: Copied " << copied_count << " images to " 
                      << images_output_dir << std::endl;
        } else {
            std::cout << "  - Images: No matching images found within " << tolerance_ms 
                      << " ms tolerance" << std::endl;
        }
        
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error accessing images directory: " << e.what() << std::endl;
    }
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
    
    // 推测images目录路径（与pointclouds_dir同级）
    fs::path pointclouds_path(pointclouds_dir);
    fs::path parent_path = pointclouds_path.parent_path();
    std::string images_dir = (parent_path / "images").string();
    
    // 检查images目录是否存在
    if (fs::exists(images_dir) && fs::is_directory(images_dir)) {
        std::cout << "Found images directory: " << images_dir << std::endl;
    } else {
        std::cout << "Images directory not found: " << images_dir << std::endl;
        std::cout << "Will skip image copying." << std::endl;
        images_dir = "";  // 清空，表示不处理图像
    }
    
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
    // std::string output_file = output_dir + "/map.pcd";
    // if (pcl::io::savePCDFileBinary(output_file, *map_cloud) == -1)
    // {
    //     std::cerr << "Could not save PCD file: " << output_file << std::endl;
    //     error_count++;
    // }

    // 固定网格大小为300x300米
    const float grid_size = 400.0f;  // 每个网格300米

    // 计算边界框
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*map_cloud, minPt, maxPt);
    
    float x_range = maxPt.x - minPt.x;
    float y_range = maxPt.y - minPt.y;
    
    // 根据地图范围自动计算需要的网格数量（向上取整）
    int grid_x_num = static_cast<int>(std::ceil(x_range / grid_size));
    int grid_y_num = static_cast<int>(std::ceil(y_range / grid_size));
    
    // 确保至少有一个网格
    grid_x_num = std::max(1, grid_x_num);
    grid_y_num = std::max(1, grid_y_num);
    
    float grid_x_size = grid_size;  // X方向网格大小
    float grid_y_size = grid_size;  // Y方向网格大小
    
    int total_grids = grid_x_num * grid_y_num;
    
    std::cout << "========== Grid Division Info ==========" << std::endl;
    std::cout << "Map range: X=" << x_range << "m, Y=" << y_range << "m" << std::endl;
    std::cout << "Grid size: " << grid_size << "m x " << grid_size << "m" << std::endl;
    std::cout << "Grid count: " << grid_x_num << " x " << grid_y_num << " = " << total_grids << " grids" << std::endl;
    std::cout << "Map bounds: X[" << minPt.x << ", " << maxPt.x << "], Y[" 
              << minPt.y << ", " << maxPt.y << "], Z[" << minPt.z << ", " << maxPt.z << "]" << std::endl;
    std::cout << "========================================" << std::endl;

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
    // for (int i = 0; i < total_grids; i=i+10) {
        if (!gridClouds[i]->points.empty()) {
            gridClouds[i]->width = gridClouds[i]->size();
            gridClouds[i]->height = 1;

            // 计算当前网格的左下角坐标（在全局坐标系中）
            int grid_x_idx = i % grid_x_num;  // 当前网格在X方向的索引
            int grid_y_idx = i / grid_x_num;  // 当前网格在Y方向的索引
            
            // 网格左下角的全局坐标
            double grid_origin_x = minPt.x + grid_x_idx * grid_x_size;
            double grid_origin_y = minPt.y + grid_y_idx * grid_y_size;
            double grid_origin_z = minPt.z;  // Z方向使用最小值作为基准
            
            // 计算网格中心点的全局坐标
            double grid_center_x = grid_origin_x  + grid_x_size / 2.0;
            double grid_center_y = grid_origin_y  + grid_y_size / 2.0;
            double grid_center_z = grid_origin_z;  // Z方向保持与左下角相同（或者可以使用点云Z的平均值）
            
            std::cout << "Grid " << i << " origin: (" 
                      << grid_origin_x << ", " << grid_origin_y << ", " << grid_origin_z << ")" << std::endl;
            std::cout << "Grid " << i << " center: (" 
                      << grid_center_x << ", " << grid_center_y << ", " << grid_center_z << ")" << std::endl;

            // 应用体素滤波，分辨率为0.25m
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(gridClouds[i]);
            const float voxel_size = 0.25f;
            voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);  // 0.25米分辨率
            voxel_filter.filter(*filtered_cloud);
            
            std::cout << "Grid " << i << ": Original points: " << gridClouds[i]->size() 
                      << ", After voxel filter: " << filtered_cloud->size() << std::endl;

            // 将点云坐标归一化到以网格中心点为原点
            pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            normalized_cloud->points.reserve(filtered_cloud->size());
            
            for (const auto& point : filtered_cloud->points) {
                pcl::PointXYZI normalized_point;
                // 相对于网格中心点的坐标
                normalized_point.x = point.x - grid_center_x;
                normalized_point.y = point.y - grid_center_y;
                normalized_point.z = point.z - grid_center_z;
                normalized_point.intensity = point.intensity;
                normalized_cloud->points.push_back(normalized_point);
            }
            
            normalized_cloud->width = normalized_cloud->size();
            normalized_cloud->height = 1;
            normalized_cloud->is_dense = filtered_cloud->is_dense;
            
            // 构造pose矩阵，将网格中心点的坐标记录到平移部分
            Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();

            // 一旦注释掉   json中记录的是 0 0 0
            pose_matrix(0, 3) = grid_center_x;  // X方向偏移（网格中心）
            pose_matrix(1, 3) = grid_center_y;  // Y方向偏移（网格中心）
            pose_matrix(2, 3) = grid_center_z;  // Z方向偏移（网格中心）
            
            // 使用clip编号作为时间戳
            double current_timestamp = static_cast<double>(i + 1);
            
            // 使用新函数保存到clip结构中（保存归一化后的点云，同时复制图像）
            saveClipData(i + 1, output_dir, normalized_cloud, pose_matrix, current_timestamp, images_dir);
            
            // 旧的保存方式（已注释）
            // std::string output_filename = output_dir +"/grid_" + std::to_string(i) + ".pcd";
            // pcl::io::savePCDFileASCII(output_filename, *filtered_cloud);
            // std::cout << "Saved grid " << i << " with " << filtered_cloud->size() << " points to " << output_filename << " (ASCII format)" << std::endl;
        }
    }

    std::cout << "Processing completed!" << std::endl;
    std::cout << "Successfully processed: " << processed_count << " files" << std::endl;
    // std::cout << "files : " << output_file << std::endl;

    return 0;
}
