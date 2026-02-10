#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

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
    std::string input_pcd_path = "/mnt/nvme0n1p2/Note_to_ROS/tools/data/keypose26.pcd";
    std::string output_dir = "/mnt/nvme0n1p2/data/pcd_grid/";
    
    // 网格划分参数
    double base_grid_size = 120.0;      // 基准网格边长(米)
    int min_points_per_grid = 100;       // 最小点数阈值,少于此值会合并到相邻网格
    double max_aspect_ratio = 1.5;     // 最大长宽比限制
    
    // 测试选项(仅使用部分点进行快速测试)
    bool use_partial_points = true;     // 是否启用部分点测试
    size_t max_test_points = 1000;      // 测试时使用的最大点数
    
    // ==================================================
    
    // 读取点云数据
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(input_pcd_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", input_pcd_path.c_str());
        return -1;
    }
    std::cout << "Original Point Cloud: " << cloud->size() << " points" << std::endl;
    
    // 创建输出目录
    mkdir(output_dir.c_str(), 0755);

    // 部分点测试模式
    if (use_partial_points && cloud->size() > max_test_points)
    {
        pcl::PointCloud<PointType>::Ptr cloud_partial(new pcl::PointCloud<PointType>);
        cloud_partial->points.reserve(max_test_points);
        for (size_t i = 0; i < max_test_points && i < cloud->size(); i++)
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
        
        std::cout << "X strip " << i << ": [" << std::fixed << std::setprecision(1) 
                  << x_boundaries[i] << ", " << x_boundaries[i + 1] << "], "
                  << x_strip_points.size() << " points, Y range [" 
                  << strip_min_y << ", " << strip_max_y << "] = " 
                  << strip_y_range << "m" << std::endl;
        
        // 计算X方向的实际宽度
        double strip_x_size = x_boundaries[i + 1] - x_boundaries[i];
        
        // 根据长宽比限制，计算Y方向的最大允许边长
        double max_y_grid_size = strip_x_size * max_aspect_ratio;
        double actual_y_grid_size = std::min(base_grid_size, max_y_grid_size);
        
        std::cout << "  X size: " << std::setprecision(1) << strip_x_size 
                  << "m, Max Y size: " << actual_y_grid_size << "m (ratio limit: " 
                  << max_aspect_ratio << ")" << std::endl;
        
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
        
        // 合并点数少的网格到相邻网格
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
                    // 检查合并后是否会违反长宽比约束
                    double merged_min_y = std::min(strip_boundaries[merge_target].min_y, strip_boundaries[k].min_y);
                    double merged_max_y = std::max(strip_boundaries[merge_target].max_y, strip_boundaries[k].max_y);
                    double merged_y_size = merged_max_y - merged_min_y;
                    double x_size = strip_boundaries[k].max_x - strip_boundaries[k].min_x;
                    double merged_aspect_ratio = std::max(x_size / merged_y_size, merged_y_size / x_size);
                    
                    // 如果合并后长宽比会超过限制，跳过合并
                    if (merged_aspect_ratio > max_aspect_ratio)
                    {
                        continue;
                    }
                    
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
                }
            }
        }
        
        // 添加到总网格列表
        grids.push_back(strip_grids);
        for (const auto& gb : strip_boundaries)
        {
            grid_boundaries.push_back(gb);
        }
        total_grids += strip_grids.size();
    }
    
    std::cout << "\nTotal grids created: " << total_grids << std::endl;
    
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
                
                // 计算网格边长
                double x_size = 0, y_size = 0;
                for (const auto& gb : grid_boundaries)
                {
                    if (gb.grid_i == i && gb.grid_j == j)
                    {
                        x_size = gb.max_x - gb.min_x;
                        y_size = gb.max_y - gb.min_y;
                        break;
                    }
                }
                
                // 生成文件名
                std::stringstream ss;
                ss << output_dir << "grid_" 
                   << std::setw(3) << std::setfill('0') << i << "_" 
                   << std::setw(3) << std::setfill('0') << j << ".pcd";
                std::string filename = ss.str();
                
                pcl::io::savePCDFileBinary(filename, *grids[i][j]);
                
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
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        
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