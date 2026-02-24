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
#include <GeographicLib/UTMUPS.hpp>

// 绘制网格边界颜色
static std::vector<cv::Scalar> colors = {
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
typedef pcl::PointXYZ PointType;

// UTM转经纬度函数
inline void utm2lla(double utm_x, double utm_y, double utm_z, double& lat, double& lon, double& alt)
{
    const int zone = 51;
    const bool northp = true;
    try {
        GeographicLib::UTMUPS::Reverse(zone, northp, utm_x, utm_y, lat, lon);
        alt = utm_z;
    } catch (const std::exception& e) {
        std::cerr << "UTM to LLA conversion error: " << e.what() << std::endl;
        lat = lon = alt = 0.0;
    }
}

// ...existing code...

// 点云网格划分类
class PointCloudGridDivider {
public:
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
    // 参数
    double base_grid_size = 100.0;
    double max_aspect_ratio = 1.25;
    int min_points_per_grid = 20;
    bool use_voxel_filter = true;
    double voxel_leaf_size = 0.5;
    size_t max_test_points = 0; // 0表示不用测试模式
    std::string output_dir = "./pcd_grid/";
    double utm_x0 = 0, utm_y0 = 0, utm_z0 = 0;

    // 结果
    std::vector<std::vector<pcl::PointCloud<PointType>::Ptr>> grids;
    std::vector<GridBoundary> grid_boundaries;
    int total_grids = 0;
    int non_empty_grids = 0;
    int total_saved_points = 0;
    double min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    double x_range = 0, y_range = 0;

    // 构造
    PointCloudGridDivider() {}

    void setUTMOrigin(double x, double y, double z) {
        utm_x0 = x; utm_y0 = y; utm_z0 = z;
    }

    void setOutputDir(const std::string& dir) {
        output_dir = dir;
    }

    // 主接口：划分并保存网格
    void divideAndSave(const pcl::PointCloud<PointType>::Ptr& cloud) {
        using namespace std;
        using namespace pcl;
        mkdir(output_dir.c_str(), 0755);

        // 体素滤波
        PointCloud<PointType>::Ptr filtered_cloud = cloud;
        if (use_voxel_filter) {
            PointCloud<PointType>::Ptr tmp(new PointCloud<PointType>);
            VoxelGrid<PointType> voxel_filter;
            voxel_filter.setInputCloud(cloud);
            voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
            voxel_filter.filter(*tmp);
            filtered_cloud = tmp;
        }

        // 测试模式
        PointCloud<PointType>::Ptr used_cloud = filtered_cloud;
        if (max_test_points > 0 && filtered_cloud->size() > max_test_points) {
            PointCloud<PointType>::Ptr tmp(new PointCloud<PointType>);
            tmp->points.reserve(max_test_points);
            for (size_t i = 0; i < max_test_points; ++i) tmp->points.push_back(filtered_cloud->points[i]);
            used_cloud = tmp;
        }

        // 计算边界
        PointType min_pt, max_pt;
        getMinMax3D(*used_cloud, min_pt, max_pt);
        min_x = min_pt.x; max_x = max_pt.x; min_y = min_pt.y; max_y = max_pt.y;
        x_range = max_x - min_x; y_range = max_y - min_y;

        // 按X排序
        std::vector<PointWithIndex> sorted_points;
        sorted_points.reserve(used_cloud->size());
        for (size_t i = 0; i < used_cloud->size(); i++) {
            sorted_points.push_back({used_cloud->points[i], i});
        }
        sort(sorted_points.begin(), sorted_points.end(), [](const PointWithIndex& a, const PointWithIndex& b) { return a.point.x < b.point.x; });

        // X方向划分
        vector<double> x_boundaries; x_boundaries.push_back(min_x);
        double current_x = min_x;
        double max_x_grid_size = base_grid_size * max_aspect_ratio;
        while (current_x < max_x) {
            double remaining_x = max_x - current_x;
            if (remaining_x <= max_x_grid_size) break;
            current_x += base_grid_size;
            x_boundaries.push_back(current_x);
        }
        x_boundaries.push_back(max_x);
        int m = (int)x_boundaries.size() - 1;

        grids.clear(); grid_boundaries.clear(); total_grids = 0;
        for (int i = 0; i < m; i++) {
            // 收集当前X条带内的所有点
            std::vector<PointWithIndex> x_strip_points;
            for (const auto& pwi : sorted_points) {
                if (pwi.point.x >= x_boundaries[i] && pwi.point.x < x_boundaries[i + 1]) x_strip_points.push_back(pwi);
            }
            if (x_strip_points.empty()) {
                grids.push_back(vector<PointCloud<PointType>::Ptr>());
                continue;
            }
            sort(x_strip_points.begin(), x_strip_points.end(), [](const PointWithIndex& a, const PointWithIndex& b) { return a.point.y < b.point.y; });
            double strip_x_size = x_boundaries[i + 1] - x_boundaries[i];
            double max_y_grid_size = strip_x_size * max_aspect_ratio;
            double actual_y_grid_size = std::min(base_grid_size, max_y_grid_size);
            vector<PointCloud<PointType>::Ptr> strip_grids;
            std::vector<GridBoundary> strip_boundaries;
            size_t point_idx = 0; int j = 0;
            while (point_idx < x_strip_points.size()) {
                double grid_start_y = x_strip_points[point_idx].point.y;
                double grid_end_y = grid_start_y + actual_y_grid_size;
                PointCloud<PointType>::Ptr grid_cloud(new PointCloud<PointType>);
                size_t start_idx = point_idx;
                while (point_idx < x_strip_points.size() && x_strip_points[point_idx].point.y < grid_end_y) {
                    grid_cloud->points.push_back(x_strip_points[point_idx].point);
                    point_idx++;
                }
                if (grid_cloud->size() > 0) {
                    double actual_min_y = x_strip_points[start_idx].point.y;
                    double actual_max_y = (point_idx > 0) ? x_strip_points[point_idx - 1].point.y : actual_min_y;
                    GridBoundary gb;
                    gb.min_x = x_boundaries[i]; gb.max_x = x_boundaries[i + 1];
                    gb.min_y = actual_min_y; gb.max_y = actual_max_y;
                    gb.grid_i = i; gb.grid_j = j; gb.point_count = grid_cloud->size();
                    strip_grids.push_back(grid_cloud);
                    strip_boundaries.push_back(gb);
                    j++;
                }
            }
            // 合并点数少的网格到相邻网格
            bool y_has_merge = true; int y_merge_round = 0;
            while (y_has_merge) {
                y_has_merge = false; y_merge_round++;
                for (size_t k = 0; k < strip_grids.size(); k++) {
                    if (strip_grids[k]->size() > 0 && strip_grids[k]->size() < min_points_per_grid) {
                        int merge_target = -1;
                        if (k > 0 && strip_grids[k - 1]->size() > 0) merge_target = k - 1;
                        else if (k + 1 < strip_grids.size() && strip_grids[k + 1]->size() > 0) merge_target = k + 1;
                        if (merge_target != -1) {
                            double merged_min_y = std::min(strip_boundaries[merge_target].min_y, strip_boundaries[k].min_y);
                            double merged_max_y = std::max(strip_boundaries[merge_target].max_y, strip_boundaries[k].max_y);
                            double merged_y_size = merged_max_y - merged_min_y;
                            double x_size = strip_boundaries[k].max_x - strip_boundaries[k].min_x;
                            double max_edge = std::max(x_size, merged_y_size);
                            double max_allowed_edge = base_grid_size * max_aspect_ratio;
                            if (max_edge <= max_allowed_edge) {
                                for (const auto& pt : strip_grids[k]->points) strip_grids[merge_target]->points.push_back(pt);
                                strip_boundaries[merge_target].min_y = merged_min_y;
                                strip_boundaries[merge_target].max_y = merged_max_y;
                                strip_boundaries[merge_target].point_count = strip_grids[merge_target]->size();
                                strip_grids[k]->points.clear();
                                strip_boundaries[k].point_count = 0;
                                y_has_merge = true;
                            }
                        }
                    }
                }
            }
            grids.push_back(strip_grids);
            for (const auto& gb : strip_boundaries) grid_boundaries.push_back(gb);
            total_grids += strip_grids.size();
        }
        // X方向合并
        int x_merge_count = 0; int merge_round = 0; bool has_merge = true;
        while (has_merge) {
            merge_round++; has_merge = false;
            for (size_t i = 0; i < grids.size(); i++) {
                for (size_t j = 0; j < grids[i].size(); j++) {
                    if (grids[i][j]->size() == 0 || grids[i][j]->size() >= min_points_per_grid) continue;
                    GridBoundary* current_gb = nullptr;
                    for (auto& gb : grid_boundaries) {
                        if (gb.grid_i == (int)i && gb.grid_j == (int)j && gb.point_count > 0) { current_gb = &gb; break; }
                    }
                    if (!current_gb) continue;
                    int merge_target_i = -1; size_t merge_target_j = 0; GridBoundary* merge_target_gb = nullptr;
                    if (i > 0) {
                        for (size_t k = 0; k < grids[i-1].size(); k++) {
                            if (grids[i-1][k]->size() == 0) continue;
                            for (auto& gb : grid_boundaries) {
                                if (gb.grid_i == (int)(i-1) && gb.grid_j == (int)k && gb.point_count > 0) {
                                    double y_overlap = std::min(current_gb->max_y, gb.max_y) - std::max(current_gb->min_y, gb.min_y);
                                    double y_gap = std::max(current_gb->min_y, gb.min_y) - std::min(current_gb->max_y, gb.max_y);
                                    if (y_overlap > 0 || y_gap < base_grid_size * 0.5) {
                                        merge_target_i = i - 1; merge_target_j = k; merge_target_gb = &gb; break;
                                    }
                                }
                            }
                            if (merge_target_i >= 0) break;
                        }
                    }
                    if (merge_target_i < 0 && i + 1 < grids.size()) {
                        for (size_t k = 0; k < grids[i+1].size(); k++) {
                            if (grids[i+1][k]->size() == 0) continue;
                            for (auto& gb : grid_boundaries) {
                                if (gb.grid_i == (int)(i+1) && gb.grid_j == (int)k && gb.point_count > 0) {
                                    double y_overlap = std::min(current_gb->max_y, gb.max_y) - std::max(current_gb->min_y, gb.min_y);
                                    double y_gap = std::max(current_gb->min_y, gb.min_y) - std::min(current_gb->max_y, gb.max_y);
                                    if (y_overlap > 0 || y_gap < base_grid_size * 0.5) {
                                        merge_target_i = i + 1; merge_target_j = k; merge_target_gb = &gb; break;
                                    }
                                }
                            }
                            if (merge_target_i >= 0) break;
                        }
                    }
                    if (merge_target_i >= 0 && merge_target_gb != nullptr) {
                        double merged_min_x = std::min(merge_target_gb->min_x, current_gb->min_x);
                        double merged_max_x = std::max(merge_target_gb->max_x, current_gb->max_x);
                        double merged_min_y = std::min(merge_target_gb->min_y, current_gb->min_y);
                        double merged_max_y = std::max(merge_target_gb->max_y, current_gb->max_y);
                        double merged_x_size = merged_max_x - merged_min_x;
                        double merged_y_size = merged_max_y - merged_min_y;
                        double max_edge = std::max(merged_x_size, merged_y_size);
                        double max_allowed_edge = base_grid_size * max_aspect_ratio;
                        if (max_edge <= max_allowed_edge) {
                            for (const auto& pt : grids[i][j]->points) grids[merge_target_i][merge_target_j]->points.push_back(pt);
                            merge_target_gb->min_x = merged_min_x;
                            merge_target_gb->max_x = merged_max_x;
                            merge_target_gb->min_y = merged_min_y;
                            merge_target_gb->max_y = merged_max_y;
                            merge_target_gb->point_count = grids[merge_target_i][merge_target_j]->size();
                            grids[i][j]->points.clear();
                            current_gb->point_count = 0;
                            x_merge_count++; has_merge = true;
                        }
                    }
                }
            }
        }
        // 保存每个网格
        total_saved_points = 0; non_empty_grids = 0;
        for (size_t i = 0; i < grids.size(); i++) {
            for (size_t j = 0; j < grids[i].size(); j++) {
                grids[i][j]->width = grids[i][j]->points.size();
                grids[i][j]->height = 1;
                grids[i][j]->is_dense = true;
                int grid_size = grids[i][j]->size();
                for (auto& gb : grid_boundaries) {
                    if (gb.grid_i == (int)i && gb.grid_j == (int)j) { gb.point_count = grid_size; break; }
                }
                if (grid_size > 0) {
                    double x_size = 0, y_size = 0, center_x = 0, center_y = 0;
                    for (const auto& gb : grid_boundaries) {
                        if (gb.grid_i == (int)i && gb.grid_j == (int)j) {
                            x_size = gb.max_x - gb.min_x; y_size = gb.max_y - gb.min_y;
                            center_x = (gb.min_x + gb.max_x) / 2.0; center_y = (gb.min_y + gb.max_y) / 2.0; break;
                        }
                    }
                    double short_edge = std::min(x_size, y_size);
                    double long_edge = std::max(x_size, y_size);
                    std::stringstream ss;
                    ss << output_dir << "grid_" << std::setw(3) << std::setfill('0') << i << "_" << std::setw(3) << std::setfill('0') << j << ".pcd";
                    std::string filename = ss.str();
                    pcl::io::savePCDFileBinary(filename, *grids[i][j]);
                    double utm_x = center_x + utm_x0;
                    double utm_y = center_y + utm_y0;
                    double latitude, longitude, altitude;
                    utm2lla(utm_x, utm_y, utm_z0, latitude, longitude, altitude);
                    std::cout << std::fixed;
                    std::cout << "Saved Grid [" << i << "," << j << "]: " << grid_size << " points, "
                              << std::setprecision(6) << latitude << " " << longitude << " "
                              << std::setprecision(0) << long_edge / 2 << " "
                              << 20.0 + long_edge / 2 << std::endl;
                    total_saved_points += grid_size; non_empty_grids++;
                }
            }
        }
    }

    // 可视化
    void visualize() {
        int img_width = 1200, img_height = 1200, margin = 100;
        cv::Mat visualization(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));
        double max_range = std::max(x_range, y_range);
        double scale = (img_width - 2 * margin) / max_range;
        auto world_to_image = [&](double x, double y) -> cv::Point {
            int img_x = margin + (x - min_x) * scale;
            int img_y = img_height - margin - (y - min_y) * scale;
            return cv::Point(img_x, img_y);
        };
        // 绘制点云
        for (const auto& gb : grid_boundaries) {
            if (gb.point_count == 0) continue;
            for (const auto& grid : grids[gb.grid_i][gb.grid_j]->points) {
                cv::Point pt = world_to_image(grid.x, grid.y);
                if (pt.x >= 0 && pt.x < img_width && pt.y >= 0 && pt.y < img_height)
                    cv::circle(visualization, pt, 1, cv::Scalar(200, 200, 200), -1);
            }
        }
        // 绘制网格
        for (const auto& gb : grid_boundaries) {
            if (gb.point_count == 0) continue;
            cv::Point top_left = world_to_image(gb.min_x, gb.max_y);
            cv::Point bottom_right = world_to_image(gb.max_x, gb.min_y);
            int color_idx = (gb.grid_i * 100 + gb.grid_j) % colors.size();
            cv::Scalar color = colors[color_idx];
            cv::rectangle(visualization, top_left, bottom_right, color, 3);
            cv::Point text_pos = world_to_image((gb.min_x + gb.max_x) / 2, (gb.min_y + gb.max_y) / 2);
            std::stringstream ss;
            ss << "[" << gb.grid_i << "," << gb.grid_j << "]";
            cv::putText(visualization, ss.str(), cv::Point(text_pos.x - 30, text_pos.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            ss.str(""); ss << gb.point_count << " pts";
            cv::putText(visualization, ss.str(), cv::Point(text_pos.x - 30, text_pos.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
        }
        cv::putText(visualization, "Grid Division - Top View", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
        std::stringstream info;
        info << "Grids: " << non_empty_grids << " | Points: " << total_saved_points << " | Avg: " << (non_empty_grids > 0 ? (int)(total_saved_points / non_empty_grids) : 0) << " pts/grid";
        cv::putText(visualization, info.str(), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        info.str("");
        info << "Output: " << output_dir;
        cv::putText(visualization, info.str(), cv::Point(20, 85), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        std::string vis_filename = output_dir + "grid_visualization.png";
        cv::imwrite(vis_filename, visualization);
        std::cout << "Saved: " << vis_filename << std::endl;
        cv::namedWindow("Grid Division", cv::WINDOW_NORMAL);
        cv::resizeWindow("Grid Division", 1200, 1200);
        cv::imshow("Grid Division", visualization);
        std::cout << "Press any key to close..." << std::endl;
        cv::waitKey(0);
    }
};
 
int main() {
    std::string input_pcd_path = "/mnt/nvme0n1p2/Note_to_ROS/tools/data/p3.pcd";
    std::string output_dir = "/mnt/nvme0n1p2/data/pcd_grid/";
    double utm_x0 = 273639.350;
    double utm_y0 = 3479137.779;
    double utm_z0 = 12.766;
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(input_pcd_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_pcd_path.c_str());
        return -1;
    }
    std::cout << "Original Point Cloud: " << cloud->size() << " points" << std::endl;
    PointCloudGridDivider divider;
    divider.base_grid_size = 100.0;
    divider.max_aspect_ratio = 1.25;
    divider.min_points_per_grid = 20;
    divider.use_voxel_filter = true;
    divider.voxel_leaf_size = 0.5;
    divider.max_test_points = 0; // 0表示不用测试模式
    divider.setOutputDir(output_dir);
    divider.setUTMOrigin(utm_x0, utm_y0, utm_z0);
    divider.divideAndSave(cloud);
    divider.visualize();
    return 0;
}