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
    std::cout << "Usage: " << argv[0] << " <work_dir>  " << std::endl;
    std::cout << "Usage:  --------------------------------------------------------" << std::endl;
    // if (argc != 2) {
    //     return -1;
    // }
    
    // std::string work_dir = argv[1];
    std::string work_dir = "/mnt/nvme0n1p2/project/tag_manfu/dwm_data/park_20251120_0/";
    
    std::string pointclouds_dir = work_dir + "/pointclouds";
    std::string poses_dir = work_dir + "/sparse/vehicle_geo_pose";
    std::string output_dir = work_dir + "/sparse/output";
    
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
    std::vector<Eigen::Vector3d> pose_positions;  // 存储所有位姿位置

    Eigen::Vector3d first_pose(0.0, 0.0, 0.0);


    int cut = 0;
    for (const auto& pcd_file : pcd_files) {
        if (cut++ % 3 != 0 )
        // if (cut++ > 3 )
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

            if (first_pose == Eigen::Vector3d(0.0, 0.0, 0.0))
            {
                first_pose = pose_data.offset_utm;
            }

            // std::cerr << "pose_data  : " << pose_data.timestamp << std::endl;
           
            // 创建完整的变换矩阵：先加偏移量到变换矩阵的平移部分
            Eigen::Affine3d full_transform = pose_data.transformation_matrix;
            
            // 记录变换后的位姿位置
            Eigen::Vector3d pose_position = full_transform.translation();
            pose_positions.push_back(pose_position);

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
    
    std::cout << "local map_cloud " << map_cloud->size()   << std::endl;
    std::cout << "first_pose " << std::fixed << std::setprecision(6) << first_pose.transpose() << std::endl;
    std::cout << "Total pose positions: " << pose_positions.size() << std::endl;

    // 为了保证精度，手动遍历每个点进行高精度变换，而不是使用Affine3d
    // 因为大数值的UTM坐标在变换时容易丢失精度
    // 保存到pcd文件 精度会丢失，保存到las文件精度不会丢失
    // std::cout << "Applying high-precision translation offset..." << std::endl;
    
    std::cout << "Translation completed. Points transformed to utm coordinate system." << std::endl;

    // ========== 基于八叉树的自适应分块和滤波 ==========
    // 自适应体素滤波参数：pair<距离阈值, 分辨率>，按距离升序排列，最后一个为最大距离
    std::vector<std::pair<float, float>> adaptive_voxel_params = {
        {0.0f, 0.025f},   // 有位姿
        {10.0f, 0.05f},   // <10m
        {20.0f, 0.1f},    // <20m
        {30.0f, 0.2f},   // >=20m
        {FLT_MAX, 0.5f}   // >=20m
    };
    
    // 1. 计算点云边界
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*map_cloud, minPt, maxPt);
    
    float x_range = maxPt.x - minPt.x;
    float y_range = maxPt.y - minPt.y;
    float z_range = maxPt.z - minPt.z;
    
    std::cout << "\n========== Octree-based Adaptive Grid Division ==========" << std::endl;
    std::cout << "Map range: X=" << x_range << "m, Y=" << y_range << "m, Z=" << z_range << "m" << std::endl;
    std::cout << "Map bounds: X[" << minPt.x << ", " << maxPt.x << "], Y[" 
              << minPt.y << ", " << maxPt.y << "], Z[" << minPt.z << ", " << maxPt.z << "]" << std::endl;
    
    // 2. 构建八叉树结构（5m分辨率）
    const float octree_resolution = 5.0f;
    
    int grid_x_num = static_cast<int>(std::ceil(x_range / octree_resolution));
    int grid_y_num = static_cast<int>(std::ceil(y_range / octree_resolution));
    int grid_z_num = static_cast<int>(std::ceil(z_range / octree_resolution));
    
    std::cout << "Octree resolution: " << octree_resolution << "m" << std::endl;
    std::cout << "Octree grid count: " << grid_x_num << " x " << grid_y_num << " x " << grid_z_num << std::endl;
    
    // 3. 数据结构：存储每个八叉树节点的点云
    struct OctreeNode {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        Eigen::Vector3f center;  // 节点中心
        bool has_pose;           // 是否包含位姿
        float min_pose_dist;     // 到最近位姿的距离
        int grid_idx;            // 网格索引
        
        OctreeNode() : cloud(new pcl::PointCloud<pcl::PointXYZI>()), 
                       has_pose(false), min_pose_dist(FLT_MAX), grid_idx(-1) {}
    };
    
    std::map<int, OctreeNode> octree_nodes;
    
    // 4. 将点分配到八叉树节点
    std::cout << "Assigning points to octree nodes..." << std::endl;
    for (const auto& point : map_cloud->points) {
        int grid_x_idx = std::min(grid_x_num - 1, (int)((point.x - minPt.x) / octree_resolution));
        int grid_y_idx = std::min(grid_y_num - 1, (int)((point.y - minPt.y) / octree_resolution));
        int grid_z_idx = std::min(grid_z_num - 1, (int)((point.z - minPt.z) / octree_resolution));
        int grid_idx = grid_z_idx * (grid_x_num * grid_y_num) + grid_y_idx * grid_x_num + grid_x_idx;
        
        // 只创建有点的节点
        if (octree_nodes.find(grid_idx) == octree_nodes.end()) {
            octree_nodes[grid_idx] = OctreeNode();
            octree_nodes[grid_idx].grid_idx = grid_idx;
            
            // 计算节点中心
            float center_x = minPt.x + (grid_x_idx + 0.5f) * octree_resolution;
            float center_y = minPt.y + (grid_y_idx + 0.5f) * octree_resolution;
            float center_z = minPt.z + (grid_z_idx + 0.5f) * octree_resolution;
            octree_nodes[grid_idx].center = Eigen::Vector3f(center_x, center_y, center_z);
        }
        
        octree_nodes[grid_idx].cloud->points.push_back(point);
    }
    
    std::cout << "Created " << octree_nodes.size() << " octree nodes (only nodes with points)" << std::endl;
    
    // 5. 检查每个节点是否包含位姿，并计算到最近位姿的距离
    std::cout << "Checking pose positions in octree nodes..." << std::endl;
    for (auto& pair : octree_nodes) {
        auto& node = pair.second;
        
        // 检查是否有位姿在这个节点内
        for (const auto& pose_pos : pose_positions) {
            Eigen::Vector3f pose_pos_f = pose_pos.cast<float>();
            
            // 检查位姿是否在当前节点内
            float half_res = octree_resolution / 2.0f;
            if (std::abs(pose_pos_f.x() - node.center.x()) <= half_res &&
                std::abs(pose_pos_f.y() - node.center.y()) <= half_res &&
                std::abs(pose_pos_f.z() - node.center.z()) <= half_res) {
                node.has_pose = true;
            }
            
            // 计算到节点中心的距离
            float dist = (pose_pos_f - node.center).norm();
            node.min_pose_dist = std::min(node.min_pose_dist, dist);
        }
    }
    
    // 6. 对每个节点应用自适应体素滤波
    std::cout << "Applying adaptive voxel filtering..." << std::endl;
    std::vector<int> node_count_per_param(adaptive_voxel_params.size(), 0);
    
    for (auto& pair : octree_nodes) {
        auto& node = pair.second;
        if (node.cloud->points.empty())
            continue;
        node.cloud->width = node.cloud->size();
        node.cloud->height = 1;
        node.cloud->is_dense = true;
        float voxel_size = 0.0f;
        size_t chosen_idx = 0;
        if (node.has_pose) {
            voxel_size = adaptive_voxel_params[0].second;
            chosen_idx = 0;
        } else {
            for (size_t idx = 1; idx < adaptive_voxel_params.size(); ++idx) {
                if (node.min_pose_dist < adaptive_voxel_params[idx].first) {
                    voxel_size = adaptive_voxel_params[idx].second;
                    chosen_idx = idx;
                    break;
                }
            }
        }
        node_count_per_param[chosen_idx]++;
        // 应用体素滤波
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(node.cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*filtered_cloud);
        node.cloud = filtered_cloud;
    }
    
    std::cout << "\nFiltering statistics:" << std::endl;
    for (size_t i = 0; i < adaptive_voxel_params.size(); ++i) {
        std::cout << "  Nodes for threshold < " << adaptive_voxel_params[i].first << "m (voxel: " << adaptive_voxel_params[i].second << "m): " << node_count_per_param[i] << std::endl;
    }
    
    // 7. 合并所有滤波后的节点点云
    std::cout << "\nMerging filtered octree nodes..." << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    int merged_count = 0;
    size_t total_filtered_points = 0;
    
    for (const auto& pair : octree_nodes) {
        const auto& node = pair.second;
        
        if (!node.cloud->points.empty()) {
            *merged_cloud += *node.cloud;
            total_filtered_points += node.cloud->size();
            merged_count++;
            
            if (merged_count % 500 == 0) {
                std::cout << "Merged " << merged_count << " octree nodes, total points: " 
                          << merged_cloud->size() << std::endl;
            }
        }
    }
    
    merged_cloud->width = merged_cloud->size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;
    
    std::cout << "Total nodes merged: " << merged_count << std::endl;
    std::cout << "Total filtered points: " << total_filtered_points << std::endl;
    std::cout << "Original points: " << map_cloud->size() << std::endl;
    std::cout << "Compression ratio: " << std::fixed << std::setprecision(2) 
              << (100.0 * total_filtered_points / map_cloud->size()) << "%" << std::endl;
    
    // 8. 保存合并后的点云
    std::string merged_filename = output_dir + "/filtered_map.pcd";
    std::cout << "\nSaving merged filtered point cloud..." << std::endl;
    pcl::io::savePCDFileBinary(merged_filename, *merged_cloud);
    std::cout << "Saved to: " << merged_filename << std::endl;

    std::cout << "Saved to: " << merged_filename << std::endl;
    
    std::cout << "\n========================================" << std::endl;

    std::cout << "Processing completed!" << std::endl;
    std::cout << "Successfully processed: " << processed_count << " files" << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;

    return 0;
}
