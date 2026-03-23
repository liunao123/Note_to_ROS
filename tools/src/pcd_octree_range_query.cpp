/**
 * pcd_octree_range_query.cpp
 *
 * 功能: 读取一个 PCD 文件，用 5m 分辨率构建八叉树 (Octree)，
 *       根据给定的查询位置，加载距离该位置一定范围内的所有点，
 *       并将结果保存为新的 PCD 文件。
 *
 * 用法:
 *   ./pcd_octree_range_query <input.pcd> <query_x> <query_y> <query_z> <search_range_m> [output.pcd]
 *
 * 参数:
 *   input.pcd       : 输入的 PCD 文件路径
 *   query_x/y/z     : 查询中心位置（与 PCD 文件坐标系相同）
 *   search_range_m  : 搜索半径，单位为米
 *   output.pcd      : （可选）输出文件路径，默认为 result.pcd
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>

// 八叉树分辨率（米）
static constexpr float OCTREE_RESOLUTION = 5.0f;

// 支持的点云类型
using PointT = pcl::PointXYZI;

/**
 * 打印使用说明
 */
void printUsage(const char* prog)
{
    std::cout << "======================================" << std::endl;
    std::cout << "PCD Octree Range Query Tool" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << prog
              << " <input.pcd> <query_x> <query_y> <query_z> <search_range_m> [output.pcd]"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  input.pcd      : Input PCD file (PointXYZI)" << std::endl;
    std::cout << "  query_x        : X coordinate of query center" << std::endl;
    std::cout << "  query_y        : Y coordinate of query center" << std::endl;
    std::cout << "  query_z        : Z coordinate of query center" << std::endl;
    std::cout << "  search_range_m : Search radius in meters" << std::endl;
    std::cout << "  output.pcd     : (optional) Output PCD file, default: result.pcd" << std::endl;
    std::cout << "======================================" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        printUsage(argv[0]);
        return -1;
    }

    // -------------------------------------------------------
    // 1. 解析命令行参数
    // -------------------------------------------------------
    const std::string input_pcd    = argv[1];
    const float       query_x      = std::stof(argv[2]);
    const float       query_y      = std::stof(argv[3]);
    const float       query_z      = std::stof(argv[4]);
    const float       search_range = std::stof(argv[5]);
    const std::string output_pcd   = (argc >= 7) ? argv[6] : "result.pcd";

    std::cout << "======================================" << std::endl;
    std::cout << "Input PCD   : " << input_pcd << std::endl;
    std::cout << "Query point : (" << query_x << ", " << query_y << ", " << query_z << ")" << std::endl;
    std::cout << "Search range: " << search_range << " m" << std::endl;
    std::cout << "Output PCD  : " << output_pcd << std::endl;
    std::cout << "Octree res. : " << OCTREE_RESOLUTION << " m" << std::endl;
    std::cout << "======================================" << std::endl;

    // -------------------------------------------------------
    // 2. 读取 PCD 文件
    // -------------------------------------------------------
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    auto t0 = std::chrono::steady_clock::now();

    std::cout << "[1/4] Loading PCD file: " << input_pcd << " ..." << std::endl;
    if (pcl::io::loadPCDFile<PointT>(input_pcd, *cloud) == -1)
    {
        std::cerr << "ERROR: Could not read PCD file: " << input_pcd << std::endl;
        return -1;
    }

    auto t1 = std::chrono::steady_clock::now();
    double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "  Loaded " << cloud->size() << " points  ("
              << load_ms << " ms)" << std::endl;

    if (cloud->empty())
    {
        std::cerr << "ERROR: The input point cloud is empty." << std::endl;
        return -1;
    }

    // -------------------------------------------------------
    // 3. 用 5m 分辨率构建八叉树
    // -------------------------------------------------------
    std::cout << "[2/4] Building octree (resolution = " << OCTREE_RESOLUTION << " m) ..." << std::endl;

    pcl::octree::OctreePointCloudSearch<PointT> octree(OCTREE_RESOLUTION);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    auto t2 = std::chrono::steady_clock::now();
    double build_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "  Octree built."
              << "  Depth: "      << octree.getTreeDepth()
              << "  Leaf nodes: " << octree.getLeafCount()
              << "  (" << build_ms << " ms)" << std::endl;

    // -------------------------------------------------------
    // 4. 半径搜索：获取给定位置一定范围内的点
    // -------------------------------------------------------
    std::cout << "[3/4] Searching points within " << search_range << " m ..." << std::endl;

    PointT query_point;
    query_point.x         = query_x;
    query_point.y         = query_y;
    query_point.z         = query_z;
    query_point.intensity = 0.0f;

    std::vector<int>   found_indices;
    std::vector<float> found_sq_distances;

    int found_count = octree.radiusSearch(
        query_point, search_range,
        found_indices, found_sq_distances);

    auto t3 = std::chrono::steady_clock::now();
    double search_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    std::cout << "  Found " << found_count << " points  (" << search_ms << " ms)" << std::endl;

    if (found_count == 0)
    {
        std::cout << "  No points found within the specified range. Exiting." << std::endl;
        return 0;
    }

    // 收集结果点云
    pcl::PointCloud<PointT>::Ptr result_cloud(new pcl::PointCloud<PointT>);
    result_cloud->reserve(found_count);

    for (int idx : found_indices)
    {
        result_cloud->points.push_back(cloud->points[idx]);
    }
    result_cloud->width    = static_cast<uint32_t>(result_cloud->points.size());
    result_cloud->height   = 1;
    result_cloud->is_dense = true;

    // 打印范围统计
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*result_cloud, min_pt, max_pt);
    std::cout << "  Result bounding box:" << std::endl;
    std::cout << "    Min: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
    std::cout << "    Max: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;

    // -------------------------------------------------------
    // 5. 保存结果
    // -------------------------------------------------------
    std::cout << "[4/4] Saving result to: " << output_pcd << " ..." << std::endl;

    if (pcl::io::savePCDFileBinary(output_pcd, *result_cloud) != 0)
    {
        std::cerr << "ERROR: Failed to save PCD file: " << output_pcd << std::endl;
        return -1;
    }

    auto t4 = std::chrono::steady_clock::now();
    double save_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
    std::cout << "  Saved " << result_cloud->size() << " points  (" << save_ms << " ms)" << std::endl;

    std::cout << "======================================" << std::endl;
    double total_ms = std::chrono::duration<double, std::milli>(t4 - t0).count();
    std::cout << "Done. Total time: " << total_ms << " ms" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}
