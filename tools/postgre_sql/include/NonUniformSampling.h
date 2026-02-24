#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <iomanip>
#include <omp.h>
#include <sstream>

#include "sensor_model.h"
#include "utils.h"
#include "fbmv/Capture.h"
#include "OctreeResample.h"
#include "tydwm/HdMapNew.hpp"
#include "obj/ObjectAnnotationLoader.h"

#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// rapidjson
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/prettywriter.h"

// 多线程实现
#include <thread>
#include <mutex>


// 非均匀采样处理类
class NonUniformSampling {
public:
  NonUniformSampling();
  ~NonUniformSampling();

  void SetSensorModel(std::shared_ptr<sensor_model> sensor_model_ptr_);
  void SetPaths(const fs::path &geoPosePath, const fs::path &imgFilePath,
                const fs::path &lidarFilePath, const fs::path &maskFilePath,
                const fs::path &hdMapPath);
  void LoadDataIndex();

  // 主要功能
  void RunRgbdMerge();
  void RunNonUniformSampling();

  // 点云块分割模板函数
  template <typename PointT>
  void MapBlock(typename pcl::PointCloud<PointT>::Ptr src, float grid_sz,
                float leaf_sz, size_t min_points_threshold);

private:
  struct SyncFrameInfo {
    size_t serialnum;
    std::unordered_map<size_t, double> timestamps_img;
    std::unordered_map<size_t, std::string> images_file;
    std::unordered_map<size_t, std::string> images_dymask_file;
    std::unordered_map<size_t, std::string> images_stmask_file;
    double timestamp_odom;
    std::string odom_pose_file;
    double timestamp_lidar;
    std::string lidar_file;
  };

  struct RgbdBlockInfo {
    std::string file_path;
    int grid_x, grid_y;
    size_t point_count;
    struct BoundingBox {
      float min_x, max_x;
      float min_y, max_y;
      std::vector<std::pair<float, float>> getCorners() const {
        return {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
      }
    } bounding_box;
  };

  std::vector<SyncFrameInfo> m_syncFramesInfo;
  std::vector<RgbdBlockInfo> m_rgbdBlockInfos;
  std::vector<std::string> m_cameraNames;
  std::vector<int> m_cameraIds;
  std::shared_ptr<sensor_model> m_sensorModel;
  std::shared_ptr<tydwm::HdMapNew> m_hdMap;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_rgbdMap;
  Eigen::Vector3d m_utmOrigin;
  bool m_hasUtmOrigin = false;

  fs::path m_geoPosePath;
  fs::path m_imgFilePath;
  fs::path m_lidarFilePath;
  fs::path m_maskFilePath;
  fs::path m_hdMapPath;
  fs::path m_outputPath;
  fs::path m_outputRgbdPath;

  std::mutex rgbd_merge_mutex2;

  // 辅助函数
  fbmv_calib::SyncFrame GetSyncFrame(size_t idx);
  pcl::PointCloud<pcl::PointXYZI>::Ptr GetLidarFrame(size_t idx);
  void RgbExtract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, size_t idx);
  bool LoadRgbdBlockInfo();
  bool ReadUtmOrigin(Eigen::Vector3f &origin_utm);
  std::vector<Triangle> Delaunay(const std::vector<cv::Point2f> &points,
                                 const cv::Rect2f &rect, float max_edge);
  void SaveMeshToPLY(const std::vector<Eigen::Vector3f> &points,
                     const std::vector<Triangle> &triangles,
                     const std::string &filename);
  
  // 动态物体移除相关
  bool IsPointInBox(const Eigen::Vector3f &pt, const tydwm::obj::AnnotatedObject &box);
  void GetBoxesFromLabelJson(const std::string &label_json,
                             std::vector<tydwm::obj::AnnotatedObject> &out_boxes,
                             bool only_keep_dynamic = false);
  void RemovePointcloudDynamicObjects_2d(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                      const std::vector<tydwm::obj::AnnotatedObject> &boxes);
  void RemovePointcloudDynamicObjects_3d(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                      const std::vector<tydwm::obj::AnnotatedObject> &boxes);
};
