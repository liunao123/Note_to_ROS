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
#include <proj.h>

#include "file_utils.hpp"

struct PoseData
{
    double timestamp;
    Eigen::Affine3d transformation_matrix;
    Eigen::Vector3d offset_utm;
};

bool init_flag = false;
double OFFSET_X = 0.0;
double OFFSET_Y = 0.0;
double OFFSET_Z = 0.0;



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


// 根据经纬度构造UTM投影的WKT2字符串
inline std::string constructUTMProjectionWKT2(double longitude, double latitude) {
    int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
    bool is_north = latitude >= 0.0;
    double central_meridian = (zone - 1) * 6.0 - 180.0 + 3.0;
    int epsg_code = is_north ? (32600 + zone) : (32700 + zone);
    std::string hemisphere = is_north ? "N" : "S";
    std::ostringstream oss;
    oss << "PROJCRS[\"WGS 84 / UTM zone " << zone << hemisphere << "\",";
    oss << "BASEGEOGCRS[\"WGS 84\",";
    oss << "ENSEMBLE[\"World Geodetic System 1984 ensemble\",";
    oss << "MEMBER[\"World Geodetic System 1984 (Transit)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G730)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G873)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G1150)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G1674)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G1762)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G2139)\"],";
    oss << "MEMBER[\"World Geodetic System 1984 (G2296)\"],";
    oss << "ELLIPSOID[\"WGS 84\",6378137,298.257223563,LENGTHUNIT[\"metre\",1]],";
    oss << "ENSEMBLEACCURACY[2.0]],";
    oss << "PRIMEM[\"Greenwich\",0,ANGLEUNIT[\"degree\",0.0174532925199433]],";
    oss << "ID[\"EPSG\",4326]],";
    oss << "CONVERSION[\"UTM zone " << zone << hemisphere << "\",";
    oss << "METHOD[\"Transverse Mercator\",ID[\"EPSG\",9807]],";
    oss << "PARAMETER[\"Latitude of natural origin\",0,ANGLEUNIT[\"degree\",0.0174532925199433],ID[\"EPSG\",8801]],";
    oss << "PARAMETER[\"Longitude of natural origin\"," << central_meridian << ",ANGLEUNIT[\"degree\",0.0174532925199433],ID[\"EPSG\",8802]],";
    oss << "PARAMETER[\"Scale factor at natural origin\",0.9996,SCALEUNIT[\"unity\",1],ID[\"EPSG\",8805]],";
    oss << "PARAMETER[\"False easting\",500000,LENGTHUNIT[\"metre\",1],ID[\"EPSG\",8806]],";
    oss << "PARAMETER[\"False northing\"," << (is_north ? 0 : 10000000) << ",LENGTHUNIT[\"metre\",1],ID[\"EPSG\",8807]]],";
    oss << "CS[Cartesian,2],";
    oss << "AXIS[\"(E)\",east,ORDER[1],LENGTHUNIT[\"metre\",1]],";
    oss << "AXIS[\"(N)\",north,ORDER[2],LENGTHUNIT[\"metre\",1]],";
    // UTM分区经度范围: 经度起始=(zone-1)*6-180, 终止=zone*6-180
    int lon_max = zone * 6 - 180;  // 最大经线 
    int lon_min = lon_max - 6 ; // 最小经线 ， -3 是中央子午线经线
    oss << "USAGE[SCOPE[\"Navigation and medium accuracy spatial referencing.\"],AREA[\"Between "
        << lon_min << "°E and " << lon_max << "°E, northern hemisphere between equator and 84°N, onshore and offshore. China. Indonesia. Japan. North Korea. Philippines. Russian Federation. South Korea. Taiwan.\"],BBOX[0," << lon_min << ",84," << lon_max << "]],";
    oss << "ID[\"EPSG\"," << epsg_code << "]]";
    // std::cout << "wkt2: " << oss.str() << std::endl;
    return oss.str();
}


// 用PDAL写LAS，参数与writePly一致
// 需在.cpp实现，或在此头文件中用宏控制实现体
template <typename PointT>
inline void writeLasWithPDAL(const pcl::PointCloud<PointT> &cloud, const std::string &output_las,
                             double offset_x, double offset_y, double offset_z, bool use_rgb,
                             double longitude = 117.0, double latitude = 39.0)
{
    /*
    // 1. 保存点云为临时PCD/PLY文件
    std::string tmp_pcd = "/tmp/tmp_pdal_input.pcd";
    pcl::PointCloud<PointT> cloud_out;
    pcl::copyPointCloud(cloud, cloud_out);
    for (auto &p : cloud_out.points) {
        p.x += offset_x;
        p.y += offset_y;
        p.z += offset_z;
    }
    pcl::io::savePCDFileBinary(tmp_pcd, cloud_out);

    // 2. 构造PDAL pipeline JSON
    std::string wkt2 = constructUTMProjectionWKT2(longitude, latitude);
    std::ostringstream pipeline;
    pipeline << R"({
        \"pipeline\": [
            {\"type\": \"readers.pcd\", \"filename\": \")" << tmp_pcd << R"(\"},
            {\"type\": \"writers.las\", \"filename\": \")" << output_las << R"(\", \"spatialreference\": \")" << wkt2 << R"(\"}
        ]
    })";

    // 3. 调用PDAL执行pipeline
    pdal::PipelineManager manager;
    std::istringstream json(pipeline.str());
    manager.readPipeline(json);
    manager.execute();

    // 4. 删除临时文件
    std::filesystem::remove(tmp_pcd);
    */
    std::cerr << "writeLasWithPDAL: 需要在.cpp中实现，或取消注释头文件中的实现体（需链接PDAL）" << std::endl;
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



// 将UTM坐标转换为经纬度
inline std::pair<double, double> convertUTMToLatLon(double utm_x, double utm_y, int zone, bool is_north)
{
    // 构造UTM投影的EPSG代码
    int epsg_code = is_north ? (32600 + zone) : (32700 + zone);
    std::string source_crs = "EPSG:" + std::to_string(epsg_code);
    
    // 创建PROJ上下文和变换
    PJ_CONTEXT *ctx = proj_context_create();
    PJ *transform = proj_create_crs_to_crs(ctx, source_crs.c_str(), "EPSG:4326", nullptr);
    
    if (!transform) {
        std::cerr << "Failed to create coordinate transformation from " << source_crs << " to EPSG:4326" << std::endl;
        proj_context_destroy(ctx);
        return std::make_pair(0.0, 0.0);
    }
    
    // 执行坐标转换
    PJ_COORD input = proj_coord(utm_x, utm_y, 0.0, 0.0);
    PJ_COORD output = proj_trans(transform, PJ_FWD, input);
    
    // 检查转换错误
    int err = proj_errno(transform);
    if (err != 0) {
        std::cerr << "UTM to LatLon conversion error: " << proj_errno_string(err) << std::endl;
    }
    
    // 清理资源
    proj_destroy(transform);
    proj_context_destroy(ctx);
    
    // 检查转换是否成功
    if (output.xyzt.x == HUGE_VAL || output.xyzt.y == HUGE_VAL || err != 0) {
        std::cerr << "Coordinate transformation failed" << std::endl;
        return std::make_pair(0.0, 0.0);
    }
    
    // PROJ返回的是(纬度, 经度)顺序，需要转换为(经度, 纬度)
    double longitude = output.xyzt.x;
    double latitude = output.xyzt.y;
    
    return std::make_pair(latitude, longitude);
}

// 将经纬度转换为UTM坐标
std::pair<double, double> convertLatLonToUTM(double longitude, double latitude)
{
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

    if (!transform)
    {
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
    if (err != 0)
    {
        std::cerr << "utm: " << proj_errno_string(err) << std::endl;
    }

    // 清理资源
    proj_destroy(transform);
    proj_context_destroy(ctx);

    // 检查转换是否成功
    if (output.xyzt.x == HUGE_VAL || output.xyzt.y == HUGE_VAL || err != 0)
    {
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
void writeLas(const pcl::PointCloud<PointT> &cloud, const std::string &output_las,
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

    // 设置WKT2格式的空间参考（根据经纬度自动生成）
    liblas::SpatialReference srs;
    std::string wkt2 = constructUTMProjectionWKT2(longitude, latitude);
    srs.SetWKT(wkt2);
    header.SetSRS(srs);
    // ...existing code...

    // 手动添加PCSCitationGeoKey VLR，确保 citation 信息写入
    // std::string pcs_citation = "WGS 84 / UTM zone 51N";
    // std::vector<uint8_t> citation_bytes(pcs_citation.begin(), pcs_citation.end());
    // if (citation_bytes.size() > 512) citation_bytes.resize(512);
    // liblas::VariableRecord vlr;
    // vlr.SetUserId("LASF_Projection");
    // vlr.SetRecordId(2112); // 2112 = PCSCitationGeoKey
    // vlr.SetDescription("PCSCitationGeoKey");
    // vlr.SetData(citation_bytes);
    // header.AddVLR(vlr);

    // 设置文件创建信息 - 使用当前系统时间
    header.SetSoftwareId("PCL to LAS Converter");

    // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm *tm_info = std::localtime(&time_t);

    // 计算当前日期是一年中的第几天
    std::tm start_of_year = *tm_info;
    start_of_year.tm_mon = 0;  // 1月 (0-based)
    start_of_year.tm_mday = 1; // 1号
    start_of_year.tm_hour = 0;
    start_of_year.tm_min = 0;
    start_of_year.tm_sec = 0;

    auto start_time = std::mktime(&start_of_year);
    auto current_time = std::mktime(tm_info);
    int day_of_year = static_cast<int>((current_time - start_time) / (24 * 3600)) + 1;

    header.SetCreationDOY(day_of_year);              // 当前是一年中的第几天
    header.SetCreationYear(tm_info->tm_year + 1900); // 当前年份

    // std::cout << "LAS file creation time: Year=" << (tm_info->tm_year + 1900)
    //           << ", Day of year=" << day_of_year << std::endl;

    std::ofstream ofs(output_las, std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    for (const auto &p : cloud.points)
    {
        liblas::Point lasPoint(&header);
        lasPoint.SetX(p.x + offset_x);
        lasPoint.SetY(p.y + offset_y);
        lasPoint.SetZ(p.z + offset_z);
        if constexpr (std::is_same<PointT, pcl::PointXYZRGB>::value)
        {
            liblas::Color color;
            color.SetRed(static_cast<uint16_t>(p.r) * 257);
            color.SetGreen(static_cast<uint16_t>(p.g) * 257);
            color.SetBlue(static_cast<uint16_t>(p.b) * 257);
            lasPoint.SetColor(color);
        }
        else if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
        {
            lasPoint.SetIntensity(static_cast<uint16_t>(p.intensity));
        }
        writer.WritePoint(lasPoint);
    }
}
