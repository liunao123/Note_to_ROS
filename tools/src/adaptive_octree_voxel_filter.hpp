#pragma once
#include <map>
#include <utility>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


template <typename PointT>
class AdaptiveOctreeVoxelFilter {
public:
    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtr = typename CloudT::Ptr;

    explicit AdaptiveOctreeVoxelFilter(float octree_resolution = 10.0f)
        : octree_resolution_(octree_resolution),
          raw_unified_cloud_(new CloudT),
          filtered_cloud_(new CloudT) {
        adaptive_voxel_params_ = {
            {0.0f, 0.025f},
            {10.0f, 0.05f},
            {20.0f, 0.1f},
            {30.0f, 0.2f},
            {FLT_MAX, 0.5f}
        };
    }

    bool setKeyPose(const std::vector<Eigen::Affine3d>& poses) {
        if (has_input_cloud_count_ && poses.size() != input_cloud_count_) {
            std::cerr << "[AdaptiveOctreeVoxelFilter] Size mismatch: poses=" << poses.size()
                      << ", clouds=" << input_cloud_count_ << std::endl;
            return false;
        }

        poses_ = poses;
        pose_positions_unified_.clear();
        has_origin_ = false;

        if (poses_.empty()) {
            origin_ = Eigen::Vector3d::Zero();
            return true;
        }

        origin_ = poses_.front().translation();
        has_origin_ = true;

        pose_positions_unified_.reserve(poses_.size());
        for (const auto& pose : poses_) {
            pose_positions_unified_.push_back(pose.translation() - origin_);
        }

        input_pose_count_ = poses_.size();
        has_input_pose_count_ = true;
        return true;
    }

    bool setKeyPointCloud(const std::vector<CloudPtr>& clouds) {
        input_cloud_count_ = clouds.size();
        has_input_cloud_count_ = true;

        if (has_input_pose_count_ && clouds.size() != input_pose_count_) {
            std::cerr << "[AdaptiveOctreeVoxelFilter] Size mismatch: poses=" << input_pose_count_
                      << ", clouds=" << clouds.size() << std::endl;
            return false;
        }

        if (!has_origin_ || poses_.empty() || clouds.size() != poses_.size()) {
            if (!poses_.empty() && clouds.size() != poses_.size()) {
                std::cerr << "[AdaptiveOctreeVoxelFilter] Size mismatch: poses=" << poses_.size()
                          << ", clouds=" << clouds.size() << std::endl;
            }
            return false;
        }

        raw_unified_cloud_.reset(new CloudT);

        for (size_t i = 0; i < clouds.size(); ++i) {
            const auto& cloud = clouds[i];
            if (!cloud) {
                continue;
            }

            CloudPtr transformed_cloud(new CloudT);
            pcl::transformPointCloud(*cloud, *transformed_cloud, poses_[i]);

            for (auto& point : transformed_cloud->points) {
                point.x -= static_cast<float>(origin_.x());
                point.y -= static_cast<float>(origin_.y());
                point.z -= static_cast<float>(origin_.z());
            }

            *raw_unified_cloud_ += *transformed_cloud;
        }

        raw_unified_cloud_->width = raw_unified_cloud_->size();
        raw_unified_cloud_->height = 1;
        raw_unified_cloud_->is_dense = true;
        return true;
    }

    void setAdaptiveVoxelParams(const std::vector<std::pair<float, float>>& adaptive_voxel_params) {
        adaptive_voxel_params_ = adaptive_voxel_params;
        std::sort(adaptive_voxel_params_.begin(), adaptive_voxel_params_.end(),
                  [](const std::pair<float, float>& lhs, const std::pair<float, float>& rhs) {
                      return lhs.first < rhs.first;
                  });
    }

    CloudPtr executeFiltering() {
        filtered_cloud_.reset(new CloudT);
        if (raw_unified_cloud_->empty()) {
            return filtered_cloud_;
        }

        if (adaptive_voxel_params_.empty()) {
            return raw_unified_cloud_;
        }

        PointT minPt, maxPt;
        pcl::getMinMax3D(*raw_unified_cloud_, minPt, maxPt);

        const float x_range = maxPt.x - minPt.x;
        const float y_range = maxPt.y - minPt.y;
        const float z_range = maxPt.z - minPt.z;

        const int grid_x_num = std::max(1, static_cast<int>(std::ceil(x_range / octree_resolution_)));
        const int grid_y_num = std::max(1, static_cast<int>(std::ceil(y_range / octree_resolution_)));
        const int grid_z_num = std::max(1, static_cast<int>(std::ceil(z_range / octree_resolution_)));

        std::map<int, OctreeNode> octree_nodes;

        for (const auto& point : raw_unified_cloud_->points) {
            const int grid_x_idx = std::min(grid_x_num - 1, std::max(0, static_cast<int>((point.x - minPt.x) / octree_resolution_)));
            const int grid_y_idx = std::min(grid_y_num - 1, std::max(0, static_cast<int>((point.y - minPt.y) / octree_resolution_)));
            const int grid_z_idx = std::min(grid_z_num - 1, std::max(0, static_cast<int>((point.z - minPt.z) / octree_resolution_)));
            const int grid_idx = grid_z_idx * (grid_x_num * grid_y_num) + grid_y_idx * grid_x_num + grid_x_idx;

            auto iter = octree_nodes.find(grid_idx);
            if (iter == octree_nodes.end()) {
                OctreeNode node;
                node.center = Eigen::Vector3f(
                    minPt.x + (grid_x_idx + 0.5f) * octree_resolution_,
                    minPt.y + (grid_y_idx + 0.5f) * octree_resolution_,
                    minPt.z + (grid_z_idx + 0.5f) * octree_resolution_);
                iter = octree_nodes.emplace(grid_idx, std::move(node)).first;
            }
            iter->second.cloud->points.push_back(point);
        }

        for (auto& pair : octree_nodes) {
            auto& node = pair.second;
            for (const auto& pose_pos : pose_positions_unified_) {
                const Eigen::Vector3f pose_pos_f = pose_pos.cast<float>();
                const float half_res = octree_resolution_ / 2.0f;
                if (std::abs(pose_pos_f.x() - node.center.x()) <= half_res &&
                    std::abs(pose_pos_f.y() - node.center.y()) <= half_res &&
                    std::abs(pose_pos_f.z() - node.center.z()) <= half_res) {
                    node.has_pose = true;
                }
                const float dist = (pose_pos_f - node.center).norm();
                node.min_pose_dist = std::min(node.min_pose_dist, dist);
            }
        }

        for (auto& pair : octree_nodes) {
            auto& node = pair.second;
            if (node.cloud->empty()) {
                continue;
            }

            node.cloud->width = node.cloud->size();
            node.cloud->height = 1;
            node.cloud->is_dense = true;

            float voxel_size = adaptive_voxel_params_.back().second;
            if (node.has_pose) {
                voxel_size = adaptive_voxel_params_[0].second;
            } else {
                for (size_t idx = 1; idx < adaptive_voxel_params_.size(); ++idx) {
                    if (node.min_pose_dist < adaptive_voxel_params_[idx].first) {
                        voxel_size = adaptive_voxel_params_[idx].second;
                        break;
                    }
                }
            }

            CloudPtr node_filtered_cloud(new CloudT);
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setInputCloud(node.cloud);
            voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
            voxel_filter.filter(*node_filtered_cloud);
            node.cloud = node_filtered_cloud;

            if (voxel_size < 0.15)
            // if (node.has_pose)
            {
                std::cout << "Node | Points Before: " << node.cloud->size() << std::endl;
                // 靠近行车道的点云，应该点数较多，// 如果点数过少，说明是空中漂浮的点，直接丢弃
                // ! 效果很好 20260319 by ln
                if (node.cloud->size() < 50 )
                {
                    std::cout << "Node | Points too less .  only : " << node.cloud->size() << std::endl;
                    std::cout << "Node | clear this node . ......  " << std::endl;
                    node.cloud->points.clear();
                    node.cloud->width = 0;
                    node.cloud->height = 1;
                    node.cloud->is_dense = true;
                    continue;
                }
                std::cout << "Node | Points Before: " << node.cloud->size() << std::endl;
                const int mean_k = 20;
                const double stddev_mul_thresh = 2.0;
                pcl::StatisticalOutlierRemoval<PointT> sor;
                sor.setInputCloud(node.cloud);
                sor.setMeanK(mean_k);
                sor.setStddevMulThresh(stddev_mul_thresh);
                sor.filter(*node.cloud);
                std::cout << "Node | Points After SOR: " << node.cloud->size() << std::endl;
            }
        }

        for (const auto& pair : octree_nodes) {
            if (!pair.second.cloud->empty()) {
                *filtered_cloud_ += *pair.second.cloud;
            }
        }

        filtered_cloud_->width = filtered_cloud_->size();
        filtered_cloud_->height = 1;
        filtered_cloud_->is_dense = true;
        return filtered_cloud_;
    }

    CloudPtr getRawUnifiedCloud() const {
        return raw_unified_cloud_;
    }

private:
    struct OctreeNode {
        CloudPtr cloud;
        Eigen::Vector3f center;
        bool has_pose;
        float min_pose_dist;

        OctreeNode()
            : cloud(new CloudT), center(Eigen::Vector3f::Zero()), has_pose(false), min_pose_dist(FLT_MAX) {}
    };

    float octree_resolution_;
    bool has_origin_ = false;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    std::vector<Eigen::Affine3d> poses_;
    std::vector<Eigen::Vector3d> pose_positions_unified_;
    std::vector<std::pair<float, float>> adaptive_voxel_params_;
    size_t input_pose_count_ = 0;
    size_t input_cloud_count_ = 0;
    bool has_input_pose_count_ = false;
    bool has_input_cloud_count_ = false;
    CloudPtr raw_unified_cloud_;
    CloudPtr filtered_cloud_;
};
