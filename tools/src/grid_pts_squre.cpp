#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

// 绘制网格边界
std::vector<cv::Scalar> colors = {
    cv::Scalar(255, 0, 0),    // 蓝色
    cv::Scalar(0, 255, 0),    // 绿色
    cv::Scalar(0, 0, 255),    // 红色
    cv::Scalar(255, 255, 0),  // 青色
    cv::Scalar(255, 0, 255),  // 品红
    cv::Scalar(0, 255, 255),  // 黄色
    cv::Scalar(128, 0, 128),  // 紫色
    cv::Scalar(255, 128, 0),  // 橙色
};

// 定义点云类型
typedef pcl::PointXYZI PointType;

// 结构体用于存储点及其索引
struct PointWithIndex {
    PointType point;
    size_t index;
};

// 结构体用于存储网格的边界
struct GridBoundary {
    double min_x, max_x, min_y, max_y;
    int grid_i, grid_j;
    int point_count;
};
 
int main()
{
    // ==================== 配置参数 ====================
    // 输入输出路径
    // 把关键帧的位置给出来，作为分布
    std::string input_pcd_path = "/mnt/nvme0n1p2/Note_to_ROS/tools/data/keypose26.pcd";
    std::string output_dir = "/mnt/nvme0n1p2/data/pcd_grid/";
    
    // 网格划分参数
    double base_grid_size = 120.0;      // 基准网格边长(米)
    double max_aspect_ratio = 1.5;     // 最大长宽比限制
    const int min_points_per_grid  = 20;   // 最小点数阈值,少于此值会合并到相邻网格
    
    // 体素滤波参数
    bool use_voxel_filter = true;      // 是否启用体素滤波
    double voxel_leaf_size = 0.50;      // 体素大小(米)
    
    // 测试选项(仅使用部分点进行快速测试)
    // bool use_partial_points = true;     // 是否启用部分点测试
    size_t max_test_points = -1000;      // 测试时使用的最大点数
    
    // ==================================================
    
    // 读取点云数据
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(input_pcd_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", input_pcd_path.c_str());
        return -1;
    }
    std::cout << "Original Point Cloud: " << cloud->size() << " points" << std::endl;
    
    // 体素滤波
    if (use_voxel_filter)
    {
        pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*cloud_filtered);
        
        std::cout << "Voxel filtered (" << voxel_leaf_size << "m): " 
                  << cloud->size() << " -> " << cloud_filtered->size() 
                  << " points (" << std::fixed << std::setprecision(1) 
                  << (100.0 * cloud_filtered->size() / cloud->size()) << "%)" << std::endl;
        
        cloud = cloud_filtered;
    }
    
    // 创建输出目录
    mkdir(output_dir.c_str(), 0755);

    // 部分点测试模式
    if (max_test_points > 0 && cloud->size() > max_test_points)
    {
        pcl::PointCloud<PointType>::Ptr cloud_partial(new pcl::PointCloud<PointType>);
        cloud_partial->points.reserve(max_test_points);
        for (size_t i = 2200; i < 2200 + max_test_points && i < cloud->size(); i++)
        {
            cloud_partial->points.push_back(cloud->points[i]);
        }
        cloud = cloud_partial;
        std::cout << "⚠ Testing mode: Using first " << cloud->size() << " points" << std::endl;
    }
    
    // 计算点云的边界
    PointType min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    double x_range = max_pt.x - min_pt.x;
    double y_range = max_pt.y - min_pt.y;
    
    std::cout << "\n=== Grid Division Parameters ===" << std::endl;
    std::cout << "Point cloud range: X=" << std::fixed << std::setprecision(1) 
              << x_range << "m, Y=" << y_range << "m" << std::endl;
    std::cout << "Base grid size: " << base_grid_size << "m" << std::endl;
    std::cout << "Max aspect ratio: " << max_aspect_ratio << std::endl;
    std::cout << "Min points per grid: " << min_points_per_grid << std::endl;
    
    // 第一步：按X坐标排序所有点
    std::vector<PointWithIndex> sorted_points;
    sorted_points.reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); i++)
    {
        PointWithIndex pwi;
        pwi.point = cloud->points[i];
        pwi.index = i;
        sorted_points.push_back(pwi);
    }
    
    // 按X坐标排序
    std::sort(sorted_points.begin(), sorted_points.end(),
              [](const PointWithIndex& a, const PointWithIndex& b) {
                  return a.point.x < b.point.x;
              });
    
    // ========== X方向：固定边长划分 ==========
    std::vector<double> x_boundaries;
    x_boundaries.push_back(min_pt.x);
    
    double current_x = min_pt.x;
    double max_x_grid_size = base_grid_size * max_aspect_ratio;  // X方向最大允许边长
    
    while (current_x < max_pt.x)
    {
        double remaining_x = max_pt.x - current_x;
        
        // 如果剩余距离小于等于最大允许边长，直接到末尾
        if (remaining_x <= max_x_grid_size)
        {
            break;
        }
        
        current_x += base_grid_size;
        x_boundaries.push_back(current_x);
    }
    x_boundaries.push_back(max_pt.x);
    
    int m = (int)x_boundaries.size() - 1;
    std::cout << "\n=== X Direction: Fixed Size Division ===" << std::endl;
    std::cout << "Created " << m << " X strips with size ~" << base_grid_size 
              << "m (max size: " << max_x_grid_size << "m)" << std::endl;
    
    // Y方向划分:对每个X条带从下到上按大小约束划分
    std::vector<std::vector<pcl::PointCloud<PointType>::Ptr>> grids;
    std::vector<GridBoundary> grid_boundaries;
    int total_grids = 0;
    
    std::cout << "\n=== Y Direction Division ===" << std::endl;
    
    for (int i = 0; i < m; i++)
    {
        // 收集当前X条带内的所有点
        std::vector<PointWithIndex> x_strip_points;
        for (const auto& pwi : sorted_points)
        {
            if (pwi.point.x >= x_boundaries[i] && pwi.point.x < x_boundaries[i + 1])
            {
                x_strip_points.push_back(pwi);
            }
        }
        
        if (x_strip_points.empty())
        {
            grids.push_back(std::vector<pcl::PointCloud<PointType>::Ptr>());
            std::cout << "X strip " << i << ": No points, skipping" << std::endl;
            continue;
        }
        
        // 在Y方向上排序
        std::sort(x_strip_points.begin(), x_strip_points.end(),
                  [](const PointWithIndex& a, const PointWithIndex& b) {
                      return a.point.y < b.point.y;
                  });
        
        // 获取当前条带内点的实际Y范围
        double strip_min_y = x_strip_points.front().point.y;
        double strip_max_y = x_strip_points.back().point.y;
        double strip_y_range = strip_max_y - strip_min_y;
        
        // std::cout << "X strip " << i << ": [" << std::fixed << std::setprecision(1) 
        //           << x_boundaries[i] << ", " << x_boundaries[i + 1] << "], "
        //           << x_strip_points.size() << " points, Y range [" 
        //           << strip_min_y << ", " << strip_max_y << "] = " 
        //           << strip_y_range << "m" << std::endl;
        
        // 计算X方向的实际宽度
        double strip_x_size = x_boundaries[i + 1] - x_boundaries[i];
        
        // 根据长宽比限制，计算Y方向的最大允许边长
        double max_y_grid_size = strip_x_size * max_aspect_ratio;
        double actual_y_grid_size = std::min(base_grid_size, max_y_grid_size);
        
        // std::cout << "  X size: " << std::setprecision(1) << strip_x_size 
        //           << "m, Max Y size: " << actual_y_grid_size << "m (ratio limit: " 
        //           << max_aspect_ratio << ")" << std::endl;
        
        // 从下到上按固定边长划分Y方向
        std::vector<pcl::PointCloud<PointType>::Ptr> strip_grids;
        std::vector<GridBoundary> strip_boundaries;
        
        size_t point_idx = 0;
        int j = 0;  // Y方向网格索引
        
        while (point_idx < x_strip_points.size())
        {
            // 当前网格的起始Y位置是当前点的Y坐标
            double grid_start_y = x_strip_points[point_idx].point.y;
            double grid_end_y = grid_start_y + actual_y_grid_size;  // 使用受限的Y网格大小
            
            // 收集这个网格范围内的所有点
            pcl::PointCloud<PointType>::Ptr grid_cloud(new pcl::PointCloud<PointType>);
            size_t start_idx = point_idx;
            
            while (point_idx < x_strip_points.size() && 
                   x_strip_points[point_idx].point.y < grid_end_y)
            {
                grid_cloud->points.push_back(x_strip_points[point_idx].point);
                point_idx++;
            }
            
            // 如果这个网格有点，保存它
            if (grid_cloud->size() > 0)
            {
                // 计算实际的Y边界（基于点的分布）
                double actual_min_y = x_strip_points[start_idx].point.y;
                double actual_max_y = (point_idx > 0) ? x_strip_points[point_idx - 1].point.y : actual_min_y;
                
                GridBoundary gb;
                gb.min_x = x_boundaries[i];
                gb.max_x = x_boundaries[i + 1];
                gb.min_y = actual_min_y;
                gb.max_y = actual_max_y;
                gb.grid_i = i;
                gb.grid_j = j;
                gb.point_count = grid_cloud->size();
                
                strip_grids.push_back(grid_cloud);
                strip_boundaries.push_back(gb);
                
                std::cout << "  Grid [" << i << "," << j << "]: " 
                          << grid_cloud->size() << " points, Y [" 
                          << std::setprecision(1) << actual_min_y << ", " 
                          << actual_max_y << "]" << std::endl;
                
                j++;
            }
        }
        
        // 合并点数少的网格到相邻网格（多轮循环直到无法合并）
        bool y_has_merge = true;
        int y_merge_round = 0;
        while (y_has_merge)
        {
            y_has_merge = false;
            y_merge_round++;
            
            for (size_t k = 0; k < strip_grids.size(); k++)
            {
                if (strip_grids[k]->size() > 0 && strip_grids[k]->size() < min_points_per_grid)
                {
                    // 找到最近的相邻网格进行合并
                    int merge_target = -1;
                    
                    // 优先合并到下方的网格
                    if (k > 0 && strip_grids[k - 1]->size() > 0)
                    {
                        merge_target = k - 1;
                    }
                    // 否则合并到上方
                    else if (k + 1 < strip_grids.size() && strip_grids[k + 1]->size() > 0)
                    {
                        merge_target = k + 1;
                    }
                
                if (merge_target != -1)
                {
                    // 检查合并后的Y方向尺寸（不检查长宽比，只检查最长边）
                    double merged_min_y = std::min(strip_boundaries[merge_target].min_y, strip_boundaries[k].min_y);
                    double merged_max_y = std::max(strip_boundaries[merge_target].max_y, strip_boundaries[k].max_y);
                    double merged_y_size = merged_max_y - merged_min_y;
                    double x_size = strip_boundaries[k].max_x - strip_boundaries[k].min_x;
                    
                    // 只检查最长边是否超过限制（不检查长宽比）
                    double max_edge = std::max(x_size, merged_y_size);
                    double max_allowed_edge = base_grid_size * max_aspect_ratio;
                    
                    // 如果合并后最长边不超过限制，就合并
                    if (max_edge <= max_allowed_edge)
                    {
                        // 合并点云
                        for (const auto& pt : strip_grids[k]->points)
                        {
                            strip_grids[merge_target]->points.push_back(pt);
                        }
                        
                        // 更新边界
                        strip_boundaries[merge_target].min_y = merged_min_y;
                        strip_boundaries[merge_target].max_y = merged_max_y;
                        strip_boundaries[merge_target].point_count = strip_grids[merge_target]->size();
                        
                        // 清空被合并的网格
                        strip_grids[k]->points.clear();
                        strip_boundaries[k].point_count = 0;
                        
                        y_has_merge = true;  // 标记本轮有合并发生
                    }
                }
            }
        }
        }  // end while for Y-direction merge
        
        // 添加到总网格列表
        grids.push_back(strip_grids);
        for (const auto& gb : strip_boundaries)
        {
            grid_boundaries.push_back(gb);
        }
        total_grids += strip_grids.size();
    }
    
    std::cout << "\nTotal grids created: " << total_grids << std::endl;
    
    // ========== X方向合并：处理点数少于min_points_per_grid的网格 ==========
    std::cout << "\n=== X Direction Merging for grids with < " << min_points_per_grid << " points ===" << std::endl;
    int x_merge_count = 0;
    int merge_round = 0;
    bool has_merge = true;
    
    // 多轮合并，直到没有可合并的网格
    while (has_merge)
    {
        merge_round++;
        has_merge = false;
        std::cout << "\n--- Merge Round " << merge_round << " ---" << std::endl;
        
        for (size_t i = 0; i < grids.size(); i++)
        {
            for (size_t j = 0; j < grids[i].size(); j++)
            {
                // 跳过空网格或已经足够大的网格
                if (grids[i][j]->size() == 0 || grids[i][j]->size() >= min_points_per_grid)
                    continue;
            
            // 找到当前网格的边界信息
            GridBoundary* current_gb = nullptr;
            for (auto& gb : grid_boundaries)
            {
                if (gb.grid_i == i && gb.grid_j == j && gb.point_count > 0)
                {
                    current_gb = &gb;
                    break;
                }
            }
            
            if (current_gb == nullptr)
                continue;
            
            // 尝试在X方向上找相邻的网格进行合并
            int merge_target_i = -1;
            size_t merge_target_j = 0;
            GridBoundary* merge_target_gb = nullptr;
            
            // 优先尝试合并到左侧（i-1）
            if (i > 0)
            {
                for (size_t k = 0; k < grids[i-1].size(); k++)
                {
                    if (grids[i-1][k]->size() == 0)
                        continue;
                    
                    // 找到对应的边界
                    for (auto& gb : grid_boundaries)
                    {
                        if (gb.grid_i == (int)(i-1) && gb.grid_j == k && gb.point_count > 0)
                        {
                            // 检查Y方向是否有重叠或相邻
                            double y_overlap = std::min(current_gb->max_y, gb.max_y) - 
                                             std::max(current_gb->min_y, gb.min_y);
                            double y_gap = std::max(current_gb->min_y, gb.min_y) - 
                                         std::min(current_gb->max_y, gb.max_y);
                            
                            // 如果Y方向有重叠或间隙不大，可以合并
                            if (y_overlap > 0 || y_gap < base_grid_size * 0.5)
                            {
                                merge_target_i = i - 1;
                                merge_target_j = k;
                                merge_target_gb = &gb;
                                break;
                            }
                        }
                    }
                    if (merge_target_i >= 0)
                        break;
                }
            }
            
            // 如果左侧没找到，尝试右侧（i+1）
            if (merge_target_i < 0 && i + 1 < grids.size())
            {
                for (size_t k = 0; k < grids[i+1].size(); k++)
                {
                    if (grids[i+1][k]->size() == 0)
                        continue;
                    
                    // 找到对应的边界
                    for (auto& gb : grid_boundaries)
                    {
                        if (gb.grid_i == (int)(i+1) && gb.grid_j == k && gb.point_count > 0)
                        {
                            // 检查Y方向是否有重叠或相邻
                            double y_overlap = std::min(current_gb->max_y, gb.max_y) - 
                                             std::max(current_gb->min_y, gb.min_y);
                            double y_gap = std::max(current_gb->min_y, gb.min_y) - 
                                         std::min(current_gb->max_y, gb.max_y);
                            
                            // 如果Y方向有重叠或间隙不大，可以合并
                            if (y_overlap > 0 || y_gap < base_grid_size * 0.5)
                            {
                                merge_target_i = i + 1;
                                merge_target_j = k;
                                merge_target_gb = &gb;
                                break;
                            }
                        }
                    }
                    if (merge_target_i >= 0)
                        break;
                }
            }
            
            // 执行合并（只检查最长边约束，不检查长宽比）
            if (merge_target_i >= 0 && merge_target_gb != nullptr)
            {
                // 计算合并后的尺寸
                double merged_min_x = std::min(merge_target_gb->min_x, current_gb->min_x);
                double merged_max_x = std::max(merge_target_gb->max_x, current_gb->max_x);
                double merged_min_y = std::min(merge_target_gb->min_y, current_gb->min_y);
                double merged_max_y = std::max(merge_target_gb->max_y, current_gb->max_y);
                
                double merged_x_size = merged_max_x - merged_min_x;
                double merged_y_size = merged_max_y - merged_min_y;
                
                // 只检查最长边是否超过限制
                double max_edge = std::max(merged_x_size, merged_y_size);
                double max_allowed_edge = base_grid_size * max_aspect_ratio;
                
                // 只检查最长边约束（不检查长宽比）
                if (max_edge <= max_allowed_edge)
                {
                    double merged_aspect_ratio = std::max(merged_x_size / merged_y_size, 
                                                          merged_y_size / merged_x_size);
                    std::cout << "  Merging grid [" << i << "," << j << "] (" 
                              << grids[i][j]->size() << " pts) into [" 
                              << merge_target_i << "," << merge_target_j << "] (" 
                              << grids[merge_target_i][merge_target_j]->size() << " pts)"
                              << " | aspect ratio: " << std::setprecision(2) << merged_aspect_ratio 
                              << ", max edge: " << std::setprecision(1) << max_edge << "m" << std::endl;
                    
                    // 合并点云
                    for (const auto& pt : grids[i][j]->points)
                    {
                        grids[merge_target_i][merge_target_j]->points.push_back(pt);
                    }
                    
                    // 更新目标网格的边界
                    merge_target_gb->min_x = merged_min_x;
                    merge_target_gb->max_x = merged_max_x;
                    merge_target_gb->min_y = merged_min_y;
                    merge_target_gb->max_y = merged_max_y;
                    merge_target_gb->point_count = grids[merge_target_i][merge_target_j]->size();
                    
                    // 清空当前网格
                    grids[i][j]->points.clear();
                    current_gb->point_count = 0;
                    
                    x_merge_count++;
                    has_merge = true;  // 标记本轮有合并发生
                }
                else
                {
                    std::cout << "  ⚠ Grid [" << i << "," << j << "] (" 
                              << grids[i][j]->size() << " pts) cannot merge: "
                              << "max edge " << std::setprecision(1) << max_edge 
                              << "m (limit: " << max_allowed_edge << "m)" << std::endl;
                }
            }
        }
    }
    }  // end while
    
    std::cout << "\nX-direction merges completed: " << x_merge_count << " grids merged in " 
              << merge_round << " rounds" << std::endl;
    
    // 统计并保存每个网格
    int total_saved_points = 0;
    int non_empty_grids = 0;
    int min_points = INT_MAX;
    int max_points = 0;
    
    // 预先计算平均值（用于输出）
    size_t total_points = cloud->size();
    double avg_points = (double)total_points / total_grids;
    
    std::cout << "\n=== Saving Grids ===" << std::endl;
    
    for (size_t i = 0; i < grids.size(); i++)
    {
        for (size_t j = 0; j < grids[i].size(); j++)
        {
            grids[i][j]->width = grids[i][j]->points.size();
            grids[i][j]->height = 1;
            grids[i][j]->is_dense = true;
            
            int grid_size = grids[i][j]->size();
            
            // 更新grid_boundaries中的点数
            for (auto& gb : grid_boundaries)
            {
                if (gb.grid_i == i && gb.grid_j == j)
                {
                    gb.point_count = grid_size;
                    break;
                }
            }
            
            if (grid_size > 0)
            {
                min_points = std::min(min_points, grid_size);
                max_points = std::max(max_points, grid_size);
                
                // 计算网格边长和中心点
                double x_size = 0, y_size = 0;
                double center_x = 0, center_y = 0;
                for (const auto& gb : grid_boundaries)
                {
                    if (gb.grid_i == i && gb.grid_j == j)
                    {
                        x_size = gb.max_x - gb.min_x;
                        y_size = gb.max_y - gb.min_y;
                        center_x = (gb.min_x + gb.max_x) / 2.0;
                        center_y = (gb.min_y + gb.max_y) / 2.0;
                        break;
                    }
                }
                
                // 确定短边和长边
                double short_edge = std::min(x_size, y_size);
                double long_edge = std::max(x_size, y_size);
                
                // 生成文件名
                std::stringstream ss;
                ss << output_dir << "grid_" 
                   << std::setw(3) << std::setfill('0') << i << "_" 
                   << std::setw(3) << std::setfill('0') << j << ".pcd";
                std::string filename = ss.str();
                
                pcl::io::savePCDFileBinary(filename, *grids[i][j]);
                
                // 输出保存的网格信息
                std::cout << "  Saved Grid [" << i << "," << j << "]: " 
                          << grid_size << " points, center: (" 
                          << std::setprecision(2) << center_x << ", " << center_y 
                          << "), size: " << std::setprecision(1) 
                          << short_edge << "m x " << long_edge << "m" << std::endl;
                
                total_saved_points += grid_size;
                non_empty_grids++;
            }
        }
    }
    
    // 计算统计信息
    double final_avg_points = (double)total_saved_points / non_empty_grids;
    double diff = max_points - min_points;
    double diff_ratio = (final_avg_points > 0) ? (diff / final_avg_points) * 100.0 : 0.0;
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Grids: " << non_empty_grids << " (total points: " << total_saved_points << ")" << std::endl;
    std::cout << "Points per grid: min=" << min_points << ", max=" << max_points 
              << ", avg=" << std::fixed << std::setprecision(1) << final_avg_points << std::endl;
    std::cout << "Balance: " << std::fixed << std::setprecision(2) 
              << diff_ratio << "% deviation" << std::endl;
    std::cout << "Output: " << output_dir << std::endl;
    
    // ==================== 可视化 ====================
    std::cout << "\n=== Visualization ===" << std::endl;
    
    int img_width = 1200;
    int img_height = 1200;
    cv::Mat visualization(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));
    
    double max_range = std::max(x_range, y_range);
    int margin = 100;
    double scale = (img_width - 2 * margin) / max_range;
    
    auto world_to_image = [&](double x, double y) -> cv::Point {
        int img_x = margin + (x - min_pt.x) * scale;
        int img_y = img_height - margin - (y - min_pt.y) * scale;
        return cv::Point(img_x, img_y);
    };
    
    // 绘制点云
    for (const auto& point : cloud->points)
    {
        cv::Point pt = world_to_image(point.x, point.y);
        if (pt.x >= 0 && pt.x < img_width && pt.y >= 0 && pt.y < img_height)
        {
            cv::circle(visualization, pt, 1, cv::Scalar(200, 200, 200), -1);
        }
    }
    

    
    for (const auto& gb : grid_boundaries)
    {
        if (gb.point_count == 0)
            continue;
            
        cv::Point top_left = world_to_image(gb.min_x, gb.max_y);
        cv::Point bottom_right = world_to_image(gb.max_x, gb.min_y);
        
        // 选择颜色
        int color_idx = (gb.grid_i * 100 + gb.grid_j) % colors.size();
        cv::Scalar color = colors[color_idx];
        
        cv::rectangle(visualization, top_left, bottom_right, color, 3);
        
        // 添加网格标注
        cv::Point text_pos = world_to_image((gb.min_x + gb.max_x) / 2, (gb.min_y + gb.max_y) / 2);
        
        std::stringstream ss;
        ss << "[" << gb.grid_i << "," << gb.grid_j << "]";
        cv::putText(visualization, ss.str(), cv::Point(text_pos.x - 30, text_pos.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        
        ss.str("");
        ss << gb.point_count << " pts";
        cv::putText(visualization, ss.str(), cv::Point(text_pos.x - 30, text_pos.y + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }
    
    // 添加标题和信息
    cv::putText(visualization, "Grid Division - Top View", cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    
    std::stringstream info;
    info << "Grids: " << non_empty_grids << " | Points: " << cloud->size() 
         << " | Avg: " << (int)final_avg_points << " pts/grid";
    cv::putText(visualization, info.str(), cv::Point(20, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    info.str("");
    info << "Balance: " << std::fixed << std::setprecision(1) << diff_ratio << "% deviation";
    cv::putText(visualization, info.str(), cv::Point(20, 85), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // 保存和显示
    std::string vis_filename = output_dir + "grid_visualization.png";
    cv::imwrite(vis_filename, visualization);
    std::cout << "Saved: " << vis_filename << std::endl;
    
    cv::namedWindow("Grid Division", cv::WINDOW_NORMAL);
    cv::resizeWindow("Grid Division", 1200, 1200);
    cv::imshow("Grid Division", visualization);
    std::cout << "Press any key to close..." << std::endl;
    cv::waitKey(0);
 
    return 0;
}