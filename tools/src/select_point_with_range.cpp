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
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <algorithm>
#include <liblas/liblas.hpp>
#include <fstream>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <ctime>
#include <chrono>
#include <proj.h>

bool init_flag  = false;
double OFFSET_X = 0.0;
double OFFSET_Y = 0.0;
double OFFSET_Z = 0.0;

// 根据经纬度构造UTM投影信息
std::string constructUTMProjectionWKT(double longitude, double latitude) {
    // 计算UTM zone号
    int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
    
    // 判断南北半球
    bool is_north = latitude >= 0.0;
    
    // 计算中央经线
    double central_meridian = (zone - 1) * 6.0 - 180.0 + 3.0;
    
    // 构造EPSG代码
    int epsg_code = is_north ? (32600 + zone) : (32700 + zone);
    
    // 构造WKT字符串
    std::string hemisphere = is_north ? "N" : "S";
    std::string wkt = "PROJCS[\"WGS 84 / UTM zone " + std::to_string(zone) + hemisphere + "\","
                      "GEOGCS[\"WGS 84\","
                      "DATUM[\"WGS_1984\","
                      "SPHEROID[\"WGS 84\",6378137,298.257223563,"
                      "AUTHORITY[\"EPSG\",\"7030\"]],"
                      "AUTHORITY[\"EPSG\",\"6326\"]],"
                      "PRIMEM[\"Greenwich\",0,"
                      "AUTHORITY[\"EPSG\",\"8901\"]],"
                      "UNIT[\"degree\",0.0174532925199433,"
                      "AUTHORITY[\"EPSG\",\"9122\"]],"
                      "AUTHORITY[\"EPSG\",\"4326\"]],"
                      "PROJECTION[\"Transverse_Mercator\"],"
                      "PARAMETER[\"latitude_of_origin\",0],"
                      "PARAMETER[\"central_meridian\"," + std::to_string(central_meridian) + "],"
                      "PARAMETER[\"scale_factor\",0.9996],"
                      "PARAMETER[\"false_easting\",500000],"
                      "PARAMETER[\"false_northing\"," + (is_north ? "0" : "10000000") + "],"
                      "UNIT[\"metre\",1,"
                      "AUTHORITY[\"EPSG\",\"9001\"]],"
                      "AXIS[\"Easting\",EAST],"
                      "AXIS[\"Northing\",NORTH],"
                      "AUTHORITY[\"EPSG\",\"" + std::to_string(epsg_code) + "\"]]";
    
    return wkt;
}

// 将经纬度转换为UTM坐标
std::pair<double, double> convertLatLonToUTM(double longitude, double latitude) {
    // 计算UTM zone号
    int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
    
    // 判断南北半球
    bool is_north = latitude >= 0.0;
    
    // 构造目标UTM投影的EPSG代码
    int epsg_code = is_north ? (32600 + zone) : (32700 + zone);
    std::string target_crs = "EPSG:" + std::to_string(epsg_code);
    // std::cerr << "target_crs   : " << target_crs << std::endl;
    // std::cerr << "longitude   : " << longitude << std::endl;
    // std::cerr << "latitude   : " << latitude << std::endl;
    
    // 创建PROJ上下文和变换
    PJ_CONTEXT *ctx = proj_context_create();
    PJ *transform = proj_create_crs_to_crs(ctx, "EPSG:4326", target_crs.c_str(), nullptr);
    // EPSG:4326 : 世界地理坐标系统（World Geodetic System 1984，简称WGS 84）的编号
    
    if (!transform) {
        std::cerr << "Failed to create coordinate transformation from EPSG:4326 to " << target_crs << std::endl;
        proj_context_destroy(ctx);
        return std::make_pair(0.0, 0.0);
    }
    
    // 执行坐标转换 - PROJ 6+版本需要归一化角度轴顺序
    // 对于EPSG:4326，输入顺序应该是 (latitude, longitude) 而不是 (longitude, latitude)
    PJ_COORD input = proj_coord(latitude, longitude, 0.0, 0.0);
    PJ_COORD output = proj_trans(transform, PJ_FWD, input);
    
    // 检查转换错误
    int err = proj_errno(transform);
    if (err != 0) {
        std::cerr << "utm: " << proj_errno_string(err) << std::endl;
    }
    
    // 清理资源
    proj_destroy(transform);
    proj_context_destroy(ctx);
    
    // 检查转换是否成功
    if (output.xyzt.x == HUGE_VAL || output.xyzt.y == HUGE_VAL || err != 0) {
        std::cerr << "Coordinate transformation failed" << std::endl;
        return std::make_pair(0.0, 0.0);
    }
    
    std::cout << "Converted Lat/Lon (" << latitude << ", " << longitude 
              << ") to UTM (" << output.xyzt.x << ", " << output.xyzt.y 
              << ") Zone " << zone << (is_north ? "N" : "S") 
              << " (EPSG:" << epsg_code << ")" << std::endl;
    
    return std::make_pair(output.xyzt.x, output.xyzt.y);
}

template <typename PointT>
void writeLas(const pcl::PointCloud<PointT>& cloud, const std::string& output_las,
              double offset_x, double offset_y, double offset_z, bool use_rgb,
              double longitude = 117.0, double latitude = 39.0)
{
    liblas::Header header;
    header.SetScale(0.01, 0.01, 0.01);
    header.SetOffset(0.0, 0.0, 0.0); // 使用传入的偏移量作为 LAS header 偏移
    if (use_rgb)
        header.SetDataFormatId(liblas::ePointFormat3);
    else
        header.SetDataFormatId(liblas::ePointFormat1);

    // 添加投影信息 - 根据经纬度动态构造UTM投影
    liblas::SpatialReference srs;
    
    // 根据经纬度构造UTM投影信息
    std::string wkt = constructUTMProjectionWKT(longitude, latitude);
    
    // 计算UTM zone号用于显示
    int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
    bool is_north = latitude >= 0.0;
    std::string hemisphere = is_north ? "N" : "S";
    
    try {
        srs.SetWKT(wkt);
        header.SetSRS(srs);
        std::cout << "Added UTM Zone " << zone << hemisphere << " projection info to LAS file" << std::endl;
        std::cout << "Reference coordinates: Longitude=" << longitude << ", Latitude=" << latitude << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Warning: Could not set spatial reference: " << e.what() << std::endl;
    }
    
    // 设置文件创建信息 - 使用当前系统时间
    header.SetSoftwareId("PCL to LAS Converter");
    
    // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_info = std::localtime(&time_t);
    
    // 计算当前日期是一年中的第几天
    std::tm start_of_year = *tm_info;
    start_of_year.tm_mon = 0;    // 1月 (0-based)
    start_of_year.tm_mday = 1;   // 1号
    start_of_year.tm_hour = 0;
    start_of_year.tm_min = 0;
    start_of_year.tm_sec = 0;
    
    auto start_time = std::mktime(&start_of_year);
    auto current_time = std::mktime(tm_info);
    int day_of_year = static_cast<int>((current_time - start_time) / (24 * 3600)) + 1;
    
    header.SetCreationDOY(day_of_year);  // 当前是一年中的第几天
    header.SetCreationYear(tm_info->tm_year + 1900);  // 当前年份
    
    // std::cout << "LAS file creation time: Year=" << (tm_info->tm_year + 1900) 
    //           << ", Day of year=" << day_of_year << std::endl;

    std::ofstream ofs(output_las, std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    for (const auto& p : cloud.points) {
        liblas::Point lasPoint(&header);
        lasPoint.SetX(p.x + offset_x);
        lasPoint.SetY(p.y + offset_y);
        lasPoint.SetZ(p.z + offset_z);
        if constexpr (std::is_same<PointT, pcl::PointXYZRGB>::value) {
            liblas::Color color;
            color.SetRed(static_cast<uint16_t>(p.r) * 257);
            color.SetGreen(static_cast<uint16_t>(p.g) * 257);
            color.SetBlue(static_cast<uint16_t>(p.b) * 257);
            lasPoint.SetColor(color);
        } else if constexpr (std::is_same<PointT, pcl::PointXYZI>::value) {
            lasPoint.SetIntensity(static_cast<uint16_t>(p.intensity));
        }
        writer.WritePoint(lasPoint);
    }
}

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

        if ( !init_flag )
        {
            init_flag = true;
            OFFSET_X = pose_data.offset_utm(0);
            OFFSET_Y = pose_data.offset_utm(1);
            OFFSET_Z = pose_data.offset_utm(2) + matrix(2, 3);
            std::cout << std::fixed << std::setprecision(3) << "OFFSET_X: " << OFFSET_X << std::endl;
            std::cout << std::fixed << std::setprecision(3) << "OFFSET_Y: " << OFFSET_Y << std::endl;
            std::cout << std::fixed << std::setprecision(3) << "OFFSET_Z: " << OFFSET_Z << std::endl;
        }
        pose_data.offset_utm = pose_data.offset_utm - Eigen::Vector3d(OFFSET_X, OFFSET_Y, OFFSET_Z);

    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file " << yaml_file << ": " << e.what() << std::endl;
        throw;
    }
    
    return pose_data;
}

// 根据timestamp构造点云文件名
std::string constructPcdFileName(const std::string& root_dir, int pose_id, double timestamp) {
    // 格式化timestamp，保留3位小数
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << timestamp;
    std::string timestamp_str = oss.str();
    
    // 构造文件名: id_timestamp.pcd
    std::string filename = std::to_string(pose_id) + "_" + timestamp_str + ".pcd";
    
    // 返回完整路径
    return root_dir + "/pointclouds/" + filename;
}

// 从文件名中提取timestamp
double extractTimestampFromFileName(const std::string& filename) {
    // 从文件名中提取timestamp部分
    // 格式: id_timestamp.pcd 或 id_timestamp.yaml
    size_t underscore_pos = filename.find('_');
    size_t dot_pos = filename.find_last_of('.');
    
    if (underscore_pos != std::string::npos && dot_pos != std::string::npos && underscore_pos < dot_pos) {
        std::string timestamp_str = filename.substr(underscore_pos + 1, dot_pos - underscore_pos - 1);
        return std::stod(timestamp_str);
    }
    return -1.0; // 无效的timestamp
}

// 根据pose文件找到对应的pcd文件
std::string findCorrespondingPcdFile(const std::string& pose_file_path, const std::vector<std::string>& all_pcd_files) {
    // 从pose文件路径中提取文件名
    std::string pose_filename = fs::path(pose_file_path).filename().string();
    double pose_timestamp = extractTimestampFromFileName(pose_filename);
    
    if (pose_timestamp < 0) {
        std::cerr << "Could not extract timestamp from pose file: " << pose_filename << std::endl;
        return "";
    }
    
    // 在pcd文件列表中查找匹配的timestamp
    for (const std::string& pcd_file : all_pcd_files) {
        std::string pcd_filename = fs::path(pcd_file).filename().string();
        double pcd_timestamp = extractTimestampFromFileName(pcd_filename);
        
        // 使用小的容差来比较浮点数
        if (std::abs(pose_timestamp - pcd_timestamp) < 0.001) {
            return pcd_file;
        }
    }
    
    std::cerr << "No corresponding PCD file found for pose file: " << pose_filename << std::endl;
    return "";
}

// 加载对应的点云文件
pcl::PointCloud<pcl::PointXYZI>::Ptr loadCorrespondingPointCloud(const std::string& pose_file_path, 
                                                                  const std::vector<std::string>& all_pcd_files) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::string pcd_file_path = findCorrespondingPcdFile(pose_file_path, all_pcd_files);
    // std::cout << "  pcd File: " << pcd_file_path << std::endl;
    
    if (pcd_file_path.empty()) {
        return cloud; // 返回空点云
    }
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path, *cloud) == -1) {
        std::cerr << "Could not read PCD file: " << pcd_file_path << std::endl;
        cloud->clear();
        return cloud;
    }
    
    // 过滤掉XY值小于3m的点
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int original_size = cloud->size();
    
    for (const auto& point : cloud->points) {
        // 计算点到原点的XY平面距离
        if ( std::fabs(point.x)  < 3.0 &&  std::fabs(point.y)  < 3.0) {
            continue;
        }
        filtered_cloud->points.push_back(point);
    }
    
    // 设置过滤后点云的属性
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    int filtered_size = filtered_cloud->size();
    // std::cout << "  Loaded point cloud: " << pcd_file_path << " (" << original_size << " points)" << std::endl;
    // std::cout << "  Filtered out " << (original_size - filtered_size) << " points with XY distance < 3m" << std::endl;
    // std::cout << "  Final point cloud size: " << filtered_size << " points" << std::endl;
    return filtered_cloud;
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
            // std::cout << "  Pose " << pose_idx << " (distance: " 
            //           << sqrt(pointRadiusSquaredDistance[i]) << "m)" << std::endl;
            
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


int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " config.yaml" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Example config.yaml content:" << std::endl;
        std::cout << "root_folder: /path/to/root_folder" << std::endl;
        std::cout << "output_las: /path/to/output.las" << std::endl;
        std::cout << "select_point_lat: 39.0" << std::endl;
        std::cout << "select_point_lon: 117.0" << std::endl;
        std::cout << "search_range: 50.0" << std::endl;
        std::cout << "voxel: 0.1" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        return -1;
    }

    YAML::Node config = YAML::LoadFile(argv[1]);
    const auto root_folder = config["root_folder"].as<std::string>();
    const auto output_las = config["output_las"].as<std::string>();
    const double select_point_lat = config["select_point_lat"].as<double>();
    const double select_point_lon = config["select_point_lon"].as<double>();
    const double search_range = config["search_range"].as<double>();
    const double voxel = config["voxel"].as<double>();
    std::string root_dir = root_folder;

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Traversing directory: " << root_dir << std::endl;
    std::cout << "output_las: " << output_las << std::endl;
    std::cout << "select_point_lat: " << select_point_lat << std::endl;
    std::cout << "select_point_lon: " << select_point_lon << std::endl;
    std::cout << "search_range: " << search_range << " m . " << std::endl;
        
    // auto utm_coords = convertLatLonToUTM(select_point_lon, select_point_lat);
    // std::cout << "Query point (UTM): " << utm_coords.first << ", " << utm_coords.second << std::endl;
    // return 0;
 
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "get all key frame  pose ...... " << std::endl;
    
    // 调用函数获取指定后缀的文件路径
    std::vector<std::string> all_pose_files = getAllFilePaths(root_dir, ".yaml");
    std::vector<std::string> all_pcd_files = getAllFilePaths(root_dir, ".pcd");
    
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
    std::string output_file = root_folder + "/mapping_pose_utm.pcd";
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
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (!pose_cloud->points.empty()) {
        // 将经纬度转换为UTM坐标，并设置为查询点
        auto utm_coords = convertLatLonToUTM(select_point_lon, select_point_lat);

        pcl::PointXYZI query_point;
        query_point.x = static_cast<float>(utm_coords.first - OFFSET_X);   // 减去偏移量以匹配本地坐标系
        query_point.y = static_cast<float>(utm_coords.second - OFFSET_Y);  // 减去偏移量以匹配本地坐标系
        query_point.z = 0.0f;  // 设置为地面高度
        query_point.intensity = 0.0f;

        std::cout << "Using specified lat/lon as query point:" << std::endl;
        std::cout << "Local Query point: (" << query_point.x << ", " << query_point.y << ", " << query_point.z << ")" << std::endl;
    
        // 半径搜索示例 - 搜索xx米范围内的位姿
        std::cout << "\n=== Radius Search ===" << std::endl;
        std::vector<int> found_indices = searchNearbyPoses(kdtree, pose_cloud, query_point, search_range, root_folder + "/select_pose_within_radius.pcd");
        
        // 使用返回的索引访问对应的pose_data，并加载对应的点云文件
        std::cout << "Found pose indices with corresponding PCD files:" << std::endl;

        for (int idx : found_indices) {
            // 构造pose文件路径（假设索引对应all_pose_files中的文件）
            if (idx < all_pose_files.size()) {
                std::string pose_file_path = all_pose_files[idx];
                // 查找并加载对应的点云文件
                pcl::PointCloud<pcl::PointXYZI>::Ptr corresponding_cloud = 
                    loadCorrespondingPointCloud(pose_file_path, all_pcd_files);
                std::cout << "  Pose File: " << pose_file_path << std::endl;
                // std::cout << "  Index: " << idx << ", Timestamp: " << all_pose_data[idx].timestamp << std::endl;
                // std::cout << "  Point Cloud Size: " << corresponding_cloud->size() << " points" << std::endl;
                // std::cout << "  Position: " << all_pose_data[idx].offset_utm.transpose() << std::endl;
                // std::cout << "  ---" << std::endl;
    
                // 创建完整的变换矩阵：将offset_utm加入到变换矩阵的平移部分
                Eigen::Affine3d full_transform = all_pose_data[idx].transformation_matrix;
                // 应用变换
                pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                // 方法1: 使用完整的变换矩阵（推荐）
                pcl::transformPointCloud(*corresponding_cloud, *transformed_cloud, full_transform);

                Eigen::Affine3d T_wl = Eigen::Affine3d::Identity();
                T_wl.translation() = all_pose_data[idx].offset_utm ;
                Eigen::Quaterniond q1(1.0, 0.0, 0.0, 0.0);
                T_wl.rotate(q1);
                // std::cout << "T_wl: " << T_wl.matrix() << std::endl << std::endl;
                pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_wl);               
                *map_cloud += *transformed_cloud;
                // std::cout << " map_cloud Point Cloud Size: " << map_cloud->size() << " points" << std::endl;
            }
        }
        std::cout << std::endl;

        // 设置点云属性
        map_cloud->width = map_cloud->points.size();
        map_cloud->height = 1;
        map_cloud->is_dense = true;
        
        std::cout << "Original map cloud size: " << map_cloud->size() << " points" << std::endl;
        
        // 过滤距离query_point超过200m的点
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        const float max_distance = search_range;  // 200米
        int original_count = map_cloud->size();
        int filtered_count = 0;
        
        for (const auto& point : map_cloud->points) {
            // 计算点到query_point的距离
            float dx = point.x - query_point.x;
            float dy = point.y - query_point.y;
            if ( std::fabs(dx) <= max_distance && std::fabs(dy) <= max_distance) {
                filtered_map_cloud->points.push_back(point);
                filtered_count++;
            }
        }
        
        // 设置过滤后点云的属性
        filtered_map_cloud->width = filtered_map_cloud->points.size();
        filtered_map_cloud->height = 1;
        filtered_map_cloud->is_dense = true;
        
        std::cout << "Filtered out " << (original_count - filtered_count) << " points beyond 200m" << std::endl;
        std::cout << "Filtered map cloud size: " << filtered_map_cloud->size() << " points" << std::endl;
        
        std::cout << select_point_lon << " " << select_point_lat << std::endl;

        writeLas(*filtered_map_cloud, output_las, OFFSET_X, OFFSET_Y, OFFSET_Z, false, select_point_lon, select_point_lat );
 
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Total files found: " << all_pose_files.size() << std::endl;
    std::cout << "Total pose_data stored: " << all_pose_data.size() << std::endl;
    std::cout << "Final Point Cloud Size (within 200m): " << filtered_map_cloud->size() << " points" << std::endl;
    std::cout << "Output_las: " << output_las << " saved successfully." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "done" << std::endl;
    
    return 0;
}
