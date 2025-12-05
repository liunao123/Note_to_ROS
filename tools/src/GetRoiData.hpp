#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

#include "file_utils.hpp"
#include "pointcloud_utils.hpp"

class GetRoiData
{
public:
    // 构造函数
    GetRoiData(const std::string& config_file);
    
    // 析构函数
    ~GetRoiData() = default;

    // 主处理流程
    bool process();

private:
    // 配置参数
    ConfigParams params_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr pose_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_cloud_;
    
    // KdTree
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_;
    
    // 文件列表
    std::vector<std::string> all_pose_files_;
    std::vector<std::string> all_pcd_files_;
    
    // 位姿数据
    std::vector<PoseData> all_pose_data_;
    
    // 查询点
    pcl::PointXYZI query_point_;

    // 私有方法
    bool loadConfig(const std::string& config_file);
    bool prepareDirectories();
    bool loadFiles();
    bool loadPoseData();
    bool buildKdTree();
    std::vector<int> searchNearbyPoses(const pcl::PointXYZI& query_point, float search_radius);
    bool processFoundPoses(const std::vector<int>& found_indices);
    bool processCopyFiles(const std::vector<int>& found_indices);
    bool filterMapCloud();
    bool saveResults();
    bool saveKML();
    std::string generateOutputFilename(const std::string& prefix, const std::string& extension);
};

// 构造函数实现
inline GetRoiData::GetRoiData(const std::string& config_file)
    : pose_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
    , map_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
    , filtered_map_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
    , kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZI>())
{
    loadConfig(config_file);
}

// 加载配置
inline bool GetRoiData::loadConfig(const std::string& config_file)
{
    std::cout << "[FUNC] Entering loadConfig() - Loading configuration from: " << config_file << std::endl;
    try {
        params_ = readConfigParams(config_file);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

// 准备目录结构
inline bool GetRoiData::prepareDirectories()
{
    std::cout << "[FUNC] Entering prepareDirectories() - Preparing directory structure" << std::endl;
    if (!copy_directory_structure(params_.root_folder, params_.keyframe_folder )) {
        std::cerr << "Failed to copy directory structure. Exiting." << std::endl;
        return false;
    }
    return true;
}

// 加载文件列表
inline bool GetRoiData::loadFiles()
{
    std::cout << "[FUNC] Entering loadFiles() - Loading all pose and PCD files" << std::endl;
    all_pose_files_ = getAllFilePaths(params_.root_folder, ".yaml");
    all_pcd_files_ = getAllFilePaths(params_.root_folder, ".pcd");
    
    if (all_pose_files_.empty() || all_pcd_files_.empty()) {
        std::cerr << "No pose or PCD files found. Exiting." << std::endl;
        return false;
    }
    
    std::cout << "Load key frame and pose:  " << std::endl;
    std::cout << "pose_files.size(): " << all_pose_files_.size() << std::endl;
    std::cout << "pcd_files.size(): " << all_pcd_files_.size() << std::endl;
    
    return true;
}

// 加载位姿数据
inline bool GetRoiData::loadPoseData()
{
    std::cout << "[FUNC] Entering loadPoseData() - Loading pose data from YAML files" << std::endl;
    float session_number = 1;
    for (const auto& file_path : all_pose_files_) {
        try {
            PoseData pose_data = readPoseFromYaml(file_path);
            
            // 获取translation向量
            Eigen::Vector3d translation = pose_data.transformation_matrix.translation() + pose_data.offset_utm;
            
            // 将translation添加到点云中
            pcl::PointXYZI point;
            point.x = static_cast<float>(translation.x());
            point.y = static_cast<float>(translation.y());
            point.z = static_cast<float>(translation.z()); // 0.0f;  // 将Z值置0
            
            // 使用 session_number  作为intensity值 区分不同的采集段pose
            point.intensity = (session_number++) * 1000.0; 
            
            pose_cloud_->points.push_back(point);
            all_pose_data_.push_back(pose_data);
        } catch (const std::exception& e) {
            std::cerr << "Error processing " << file_path << ": " << e.what() << std::endl;
        }
    }
    
    // 设置点云属性
    pose_cloud_->width = pose_cloud_->points.size();
    pose_cloud_->height = 1;
    pose_cloud_->is_dense = true;
    
    // 保存点云到文件
    std::string output_file = params_.output_folder + "utm_keyframe_pose.pcd";
    if (pcl::io::savePCDFile(output_file, *pose_cloud_) == -1) {
        std::cerr << "Could not save PCD file: " << output_file << std::endl;
        return false;
    }
    
    std::cout << "keyframe saved to: " << output_file << std::endl;
    return true;
}

// 构建KdTree
inline bool GetRoiData::buildKdTree()
{
    std::cout << "[FUNC] Entering buildKdTree() - Building KdTree for spatial search" << std::endl;
    std::cout << "start search knn ...... " << std::endl;
    kdtree_->setInputCloud(pose_cloud_);
    std::cout << "KdTree built successfully with " << pose_cloud_->points.size() << " poses" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    return true;
}

// 搜索附近的位姿
inline std::vector<int> GetRoiData::searchNearbyPoses(const pcl::PointXYZI& query_point, float search_radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    if (kdtree_->radiusSearch(query_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        std::cout << "Found " << pointIdxRadiusSearch.size() << " poses within " << search_radius << "m radius" << std::endl;
        
        // 创建新的点云来存储搜索结果
        pcl::PointCloud<pcl::PointXYZI>::Ptr search_result_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            int pose_idx = pointIdxRadiusSearch[i];
            auto original_point = pose_cloud_->points[pose_idx];
            original_point.intensity = 1.0;
            search_result_cloud->points.push_back(original_point);
        }
        
        // 设置点云属性
        search_result_cloud->width = search_result_cloud->points.size();
        search_result_cloud->height = 1;
        search_result_cloud->is_dense = true;
        
        // 保存搜索结果
        std::string output_filename = params_.output_folder + "utm_select_pose_within_radius.pcd";
        if (pcl::io::savePCDFile(output_filename, *search_result_cloud) == -1) {
            std::cerr << "Could not save search result PCD file: " << output_filename << std::endl;
        } else {
            std::cout << "Search poses saved to: " << output_filename << std::endl << std::endl;
        }
    } else {
        std::cout << "No poses found within " << search_radius << "m radius" << std::endl;
    }
    
    return pointIdxRadiusSearch;
}

// 处理找到的位姿
inline bool GetRoiData::processCopyFiles(const std::vector<int>& found_indices)
{
    std::cout << "[FUNC] Entering processCopyFiles() - Copying " << found_indices.size() << " related files" << std::endl;
    int count = 0;
    for (int idx : found_indices) {
        if (idx < all_pose_files_.size()) {
            std::string pose_file_path = all_pose_files_[idx];
            // 查找相关文件并复制到输出目录
            // * done
            auto absolut_files = findRelatedFiles(pose_file_path);
            if (!absolut_files.empty()) {
                // std::cout << " pose_file_path: " << pose_file_path << "  ...... " << std::endl;
                // std::cout << " pose_file_path: " << params_.output_folder << "  ...... " << std::endl;
                absolut_files.push_back(pose_file_path);
                copyRelatedFiles(absolut_files, params_.keyframe_folder , params_);
            }
        }
    }
    return true;
}



inline bool GetRoiData::processFoundPoses(const std::vector<int>& found_indices)
{
    std::cout << "[FUNC] Entering processFoundPoses() - Processing " << found_indices.size() << " found poses" << std::endl;
    if (found_indices.empty()) {
        std::cerr << "\n\n----------------------*************--------------------" << std::endl;
        std::cerr << "   No poses found within the specified search range.    " << std::endl;
        std::cerr << "----------------------*************--------------------\n\n" << std::endl;
        return false;
    }
    
    int count = 0;
    for (int idx : found_indices) {
        if (idx < all_pose_files_.size()) {
            std::string pose_file_path = all_pose_files_[idx];
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr corresponding_cloud = 
                loadCorrespondingPointCloud(pose_file_path, all_pcd_files_);
            
            if (count++ % 50 == 0) {
                std::cout << "Accumulative keyframe : " << pose_file_path << "  ------>  " <<
                100 * count / found_indices.size() << "%" << std::endl;
            }
            
            // 创建完整的变换矩阵
            Eigen::Affine3d full_transform = all_pose_data_[idx].transformation_matrix;
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*corresponding_cloud, *transformed_cloud, full_transform);
            
            Eigen::Affine3d T_wl = Eigen::Affine3d::Identity();
            T_wl.translation() = all_pose_data_[idx].offset_utm;
            Eigen::Quaterniond q1(1.0, 0.0, 0.0, 0.0);
            T_wl.rotate(q1);
            
            pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_wl);
            *map_cloud_ += *transformed_cloud;
        }
    }
    
    std::cout << std::endl;
    
    // 设置点云属性
    map_cloud_->width = map_cloud_->points.size();
    map_cloud_->height = 1;
    map_cloud_->is_dense = true;
    
    std::cout << "Original map cloud size: " << map_cloud_->size() << " points" << std::endl;
    
    return true;
}

// 过滤地图点云
inline bool GetRoiData::filterMapCloud()
{
    std::cout << "[FUNC] Entering filterMapCloud() - Filtering map cloud within search range" << std::endl;
    filtered_map_cloud_->clear();
    const float max_distance = params_.search_range;
    int original_count = map_cloud_->size();
    int filtered_count = 0;
    
    for (const auto& point : map_cloud_->points) {
        float dx = point.x - query_point_.x;
        float dy = point.y - query_point_.y;
        if (std::fabs(dx) <= max_distance && std::fabs(dy) <= max_distance) {
            filtered_map_cloud_->points.push_back(point);
            filtered_count++;
        }
    }
    
    // 设置过滤后点云的属性
    filtered_map_cloud_->width = filtered_map_cloud_->points.size();
    filtered_map_cloud_->height = 1;
    filtered_map_cloud_->is_dense = true;
    
    std::cout << "Filtered out " << (original_count - filtered_count) << " points beyond search_range" << std::endl;
    std::cout << "Filtered map cloud size: " << filtered_map_cloud_->size() << " points" << std::endl;
    
    return true;
}

// 生成输出文件名
inline std::string GetRoiData::generateOutputFilename(const std::string& prefix, const std::string& extension)
{
    std::ostringstream coord_stream;
    coord_stream << std::fixed << std::setprecision(6) 
                 << params_.select_point_lon << "_" << params_.select_point_lat;
    std::string coord_string = coord_stream.str();
    
    return params_.output_folder + prefix + coord_string + extension;
}

// 保存结果
inline bool GetRoiData::saveResults()
{
    std::cout << "[FUNC] Entering saveResults() - Saving filtered map cloud to LAS/PLY" << std::endl;

    if (params_.enable_save_las)
    {
        std::string output_las = generateOutputFilename("utm_", ".las");
        std::cout << "Done: Output file with UTM coordinates: " << output_las << std::endl;
        writeLas(*filtered_map_cloud_, output_las, OFFSET_X, OFFSET_Y, OFFSET_Z, false,
                 params_.select_point_lon, params_.select_point_lat);
    }

    if (params_.enable_save_pcd)
    {
        std::string output_pcd = generateOutputFilename("local_", ".pcd");
        writePcd(*filtered_map_cloud_, output_pcd);
        std::cout << "Output file with local coordinates: " << output_pcd << std::endl;
    }

    if (params_.enable_save_ply)
    {
        std::string output_ply = generateOutputFilename("local_", ".ply");
        writePly(*filtered_map_cloud_, output_ply);
        std::cout << "Output file with local coordinates: " << output_ply << std::endl;
    }

    if (params_.enable_save_ply || params_.enable_save_pcd)
    {
        std::cout << "OFFSET_X: " << std::fixed << std::setprecision(3) << OFFSET_X << std::endl;
        std::cout << "OFFSET_Y: " << std::fixed << std::setprecision(3) << OFFSET_Y << std::endl;
        std::cout << "OFFSET_Z: " << std::fixed << std::setprecision(3) << OFFSET_Z << std::endl;
        writeOffsetFile(OFFSET_X, OFFSET_Y, OFFSET_Z, params_.output_folder + "local_offset.yaml");
        std::cout << "writeOffsetFile ok" << std::endl;
    }

    return true;
}

// 保存KML文件
inline bool GetRoiData::saveKML()
{
    std::cout << "[FUNC] Entering saveKML() - Saving KML file with pose locations" << std::endl;
    std::string output_kml = generateOutputFilename("utm_keyframe_pose_", ".kml");
    std::ofstream kml_file(output_kml);
    
    if (!kml_file.is_open()) {
        std::cerr << "Failed to create KML file: " << output_kml << std::endl;
        return false;
    }
    
    // 计算UTM zone（假设所有点在同一zone）
    int zone = static_cast<int>((params_.select_point_lon + 180.0) / 6.0) + 1;
    bool is_north = params_.select_point_lat >= 0.0;
    
    // 写入KML头部
    kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kml_file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    kml_file << "  <Document>\n";
    kml_file << "    <name>Pose Locations</name>\n";
    kml_file << "    <description>UTM to LatLon converted poses</description>\n";
    
    // 定义样式
    kml_file << "    <Style id=\"poseStyle\">\n";
    kml_file << "      <IconStyle>\n";
    kml_file << "        <color>ff0000ff</color>\n";  // 红色
    kml_file << "        <scale>0.5</scale>\n";
    kml_file << "        <Icon>\n";
    kml_file << "          <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n";
    kml_file << "        </Icon>\n";
    kml_file << "      </IconStyle>\n";
    kml_file << "    </Style>\n";
    
    // 写入查询点
    auto query_utm_x = query_point_.x + OFFSET_X;
    auto query_utm_y = query_point_.y + OFFSET_Y;
    auto query_latlon = convertUTMToLatLon(query_utm_x, query_utm_y, zone, is_north);
    
    kml_file << "    <Placemark>\n";
    kml_file << "      <name>Query Point</name>\n";
    kml_file << "      <description>Search center point</description>\n";
    kml_file << "      <styleUrl>#poseStyle</styleUrl>\n";
    kml_file << "      <Point>\n";
    kml_file << "        <coordinates>" << std::fixed << std::setprecision(8) 
             << query_latlon.first << "," << query_latlon.second << ",0</coordinates>\n";
    kml_file << "      </Point>\n";
    kml_file << "    </Placemark>\n";
    
    // 写入所有位姿点
    for (size_t i = 0; i < pose_cloud_->points.size(); i=i+10) {
        const auto& point = pose_cloud_->points[i];
        
        // UTM坐标需要加上偏移量
        double utm_x = point.x + OFFSET_X;
        double utm_y = point.y + OFFSET_Y;
        
        // 转换为经纬度
        auto latlon = convertUTMToLatLon(utm_x, utm_y, zone, is_north);
        
        kml_file << "    <Placemark>\n";
        kml_file << "      <name>" << i << "</name>\n";
        kml_file << "      <description>";
        kml_file << "UTM: (" << std::fixed << std::setprecision(2) << utm_x << ", " << utm_y << ")\\n";
        kml_file << "Timestamp: " << all_pose_data_[i].timestamp;
        kml_file << "</description>\n";
        kml_file << "      <Point>\n";
        kml_file << "        <coordinates>" << std::fixed << std::setprecision(8) 
                 << latlon.first << "," << latlon.second << ",0</coordinates>\n";
        kml_file << "      </Point>\n";
        kml_file << "    </Placemark>\n";
    }
    
    // KML尾部
    kml_file << "  </Document>\n";
    kml_file << "</kml>\n";
    
    kml_file.close();
    
    std::cout << "KML file saved to: " << output_kml << std::endl;
    return true;
}

// 主处理流程
inline bool GetRoiData::process()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "[FUNC] Entering process() - Starting main processing pipeline" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // 1. 准备目录
    if (!prepareDirectories()) {
        return false;
    }
    
    // 2. 加载文件
    if (!loadFiles()) {
        return false;
    }
    
    // 3. 加载位姿数据
    if (!loadPoseData()) {
        return false;
    }
    
    // 4. 构建KdTree
    if (!buildKdTree()) {
        return false;
    }
    
    // 5. 设置查询点
    if (pose_cloud_->points.empty()) {
        std::cerr << "No pose data available" << std::endl;
        return false;
    }

    auto utm_coords = convertLatLonToUTM(params_.select_point_lon, params_.select_point_lat);
    query_point_.x = static_cast<float>( utm_coords.first );
    query_point_.y = static_cast<float>( utm_coords.second );
    query_point_.z = 0.0f; //查找的时候只考虑平面距离
    query_point_.intensity = 0.0f;
    
    std::cout << "Using specified lat/lon as query point:" << std::endl;
    std::cout << "Local Query point: (" << query_point_.x << ", " << query_point_.y << ", " << query_point_.z << ")" << std::endl;
    
    // 6. 半径搜索
    std::cout << "\n=== Radius Search ===" << std::endl;
    std::vector<int> found_indices = searchNearbyPoses(query_point_, params_.search_range);
    
    // 7. 处理找到的位姿
    std::cout << "Found pose indices with corresponding PCD files:" << std::endl;
    if (!processFoundPoses(found_indices)) {
        return false;
    }

    auto result = processCopyFiles(found_indices);


    // todo 根据找到的位姿，加载对应的图像和其他数据
    
    // 8. 过滤地图点云
    if (!filterMapCloud()) {
        return false;
    }
    
    // 9. 保存结果
    if (!saveResults()) {
        return false;
    }
    
    // 10. 保存KML文件
    if (params_.enable_save_kml) {
        if (!saveKML()) {
            return false;
        }
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "done" << std::endl;
    
    return true;
}
