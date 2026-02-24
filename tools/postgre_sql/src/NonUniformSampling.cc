#include "NonUniformSampling.h"

NonUniformSampling::NonUniformSampling() {
  m_hdMap = std::make_shared<tydwm::HdMapNew>();
  m_rgbdMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::cout << "NonUniformSampling constructed." << std::endl;
}

NonUniformSampling::~NonUniformSampling() {
  std::cout << "NonUniformSampling destructed." << std::endl;
}

void NonUniformSampling::SetSensorModel(
    std::shared_ptr<sensor_model> sensor_model_ptr_) {
  this->m_sensorModel = sensor_model_ptr_;
  int id = 0;
  for (const auto &[name, param] : this->m_sensorModel->camera_params) {
    m_cameraNames.emplace_back(name);
    m_cameraIds.emplace_back(id++);
  }
}

void NonUniformSampling::SetPaths(const fs::path &geoPosePath,
                                  const fs::path &imgFilePath,
                                  const fs::path &lidarFilePath,
                                  const fs::path &maskFilePath,
                                  const fs::path &hdMapPath) {
  m_geoPosePath = geoPosePath;
  m_imgFilePath = imgFilePath;
  m_lidarFilePath = lidarFilePath;
  m_maskFilePath = maskFilePath;
  m_hdMapPath = hdMapPath;

  m_outputPath = m_geoPosePath.parent_path() / "post";
  m_outputRgbdPath = m_outputPath / "rgbd_blocks";
  if (!fs::exists(m_outputPath)) {
    fs::create_directories(m_outputPath);
  }
  if (!fs::exists(m_outputRgbdPath)) {
    fs::create_directories(m_outputRgbdPath);
  }
}

void NonUniformSampling::RunRgbdMerge() {

  size_t total = m_syncFramesInfo.size();
  // unsigned int max_threads = 1;
  unsigned int max_threads = std::thread::hardware_concurrency();
  if (max_threads == 0) max_threads = 8;
  size_t chunk = (total + max_threads - 1) / max_threads;
  std::vector<std::thread> threads;

  auto process_func = [&](size_t start, size_t end) {
    for (size_t i = start; i < end; ++i) {
      // if (i > 50)
      // {
      //   continue;
      // }
      // if (i % 10 != 0)
      // {
      //   continue;
      // }
      auto t_start = std::chrono::high_resolution_clock::now();
      std::cout << "Processing frame " << i << " for RGB-D merge." << std::endl;
      fbmv_calib::SyncFrame syncData;
      {
        syncData = GetSyncFrame(i);
      }
      const Eigen::Matrix4d &T = syncData.odom_pose * this->m_sensorModel->get_T_lv();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd(new pcl::PointCloud<pcl::PointXYZRGB>());
      RgbExtract(rgbd, i);
      if (rgbd->points.size() > 0) {
        pcl::PointCloud<pcl::PointXYZRGB> utm_cloud_temp;
        pcl::transformPointCloud(*rgbd, utm_cloud_temp, T.cast<float>());
        Eigen::Matrix4d to_utm = Eigen::Matrix4d::Identity();
        pcl::PointCloud<pcl::PointXYZRGB> utm_cloud;
        to_utm.block<3,1>(0,3) = syncData.offset_utm;
        pcl::transformPointCloud(utm_cloud_temp, utm_cloud, to_utm.cast<float>());
        {
          std::lock_guard<std::mutex> lock(this->rgbd_merge_mutex2);
          *m_rgbdMap += utm_cloud;
        }
      }
      auto t_end = std::chrono::high_resolution_clock::now();
      double elapsed = std::chrono::duration<double>(t_end - t_start).count();
      std::cout << "Frame " << i << " cost: " << elapsed << " s" << std::endl;
    }
  };

  for (unsigned int t = 0; t < max_threads; ++t) {
    size_t start = t * chunk;
    size_t end = std::min(start + chunk, total);
    if (start >= end) break;
    threads.emplace_back(process_func, start, end);
  }
  for (auto &th : threads) th.join();

  float grid_sz = 50;
  float leaf_sz = 0.05f;
  size_t min_points = 1000;
  // 注释掉了 写pcd的逻辑
  MapBlock<pcl::PointXYZRGB>(m_rgbdMap, grid_sz, leaf_sz, min_points);
}

void NonUniformSampling::RunNonUniformSampling() {
  if (!LoadRgbdBlockInfo()) {
    return;
  }

  // 1. 从HDMap上取所有路面点数据
  Eigen::Vector3f origin_utm;
  if (!ReadUtmOrigin(origin_utm)) {
    return;
  }
  m_hdMap->setLocalOrigin(origin_utm);
  m_hdMap->Load(m_hdMapPath);
  std::vector<Eigen::Vector3f> hdmapPointsVec =
      m_hdMap->GetPoints({"lane", "boundary"});
  
  // 1.1 行车道三角剖分
  std::vector<Triangle> triangles;
  {
    std::vector<cv::Point2f> road2dpoints;
    cv::Point2f minPt(std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
    cv::Point2f maxPt(std::numeric_limits<float>::min(),
                      std::numeric_limits<float>::min());

    for (const auto &pt : hdmapPointsVec) {
      cv::Point2f p(pt[0], pt[1]);
      minPt.x = std::min(minPt.x, p.x);
      minPt.y = std::min(minPt.y, p.y);
      maxPt.x = std::max(maxPt.x, p.x);
      maxPt.y = std::max(maxPt.y, p.y);
      road2dpoints.push_back(p);
    }

    cv::Rect2f rect(minPt - cv::Point2f(1, 1), maxPt + cv::Point2f(1, 1));
    triangles = Delaunay(road2dpoints, rect, 8.f);
    SaveMeshToPLY(hdmapPointsVec, triangles, "output_mesh.ply");
  }

  // 2. 使用 所有 点云 合并成的 m_rgbdMap
  // 3. 构成八叉树
  float voxel_size = 5.0f;
  OctreeResample<pcl::PointXYZRGB> octree_filter(voxel_size);
  octree_filter.setInputCloud(m_rgbdMap);
  octree_filter.setRoadMeshTriangles(triangles);
  octree_filter.buildOctree();

  // 打印各层体素数量 & 八叉树结构
  octree_filter.printVoxelCounts();
  octree_filter.printOctreeStructure();

  // 4. 八叉树非均匀采样
  std::vector<std::pair<float, float>> thresholds = {
      {5.f, 0.05f}, {10.f, 0.1f}, {20.f, 0.2f},
      {40.f, 0.4f}, {80.f, 0.8f},  {500.f, 1.6f}};
  std::cout << hdmapPointsVec.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud =
      octree_filter.resample(hdmapPointsVec, thresholds);
  std::cout << "Total rgbd points after non-uniform sampling: "
            << new_cloud->size() << std::endl;
  std::cout << "save to file: "
            << m_outputPath / "gs_init.ply" << std::endl;
  pcl::io::savePLYFileBinary(m_outputPath / "gs_init.ply", *new_cloud);

  // 利用BoxFilter，只保留XY绝对值都在100以内的点
  pcl::CropBox<pcl::PointXYZRGB> box_filter;
  box_filter.setInputCloud(new_cloud);
  const float range_xy = 100.0f;
  box_filter.setMin(Eigen::Vector4f(-range_xy, -range_xy, -1e6, 1.0f));
  box_filter.setMax(Eigen::Vector4f(range_xy, range_xy, 1e6, 1.0f));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  box_filter.filter(*cropped_cloud);
  std::cout << "save cropped file: " << (m_outputPath / "gs_init_cropped_100m.ply") << std::endl;
  pcl::io::savePLYFileBinary(m_outputPath / "gs_init_cropped_100m.ply", *cropped_cloud);
}

void NonUniformSampling::LoadDataIndex() {
  std::vector<fs::path> geoPoseFiles;
  for (const auto &entry : fs::directory_iterator(m_geoPosePath)) {
    if (entry.path().extension() == ".yaml") {
      geoPoseFiles.push_back(entry.path());
    }
  }
  
  std::sort(geoPoseFiles.begin(), geoPoseFiles.end(),
            [](const fs::path &a, const fs::path &b) {
              std::string a_stem = a.stem().string();
              std::string b_stem = b.stem().string();
              size_t a_underscore = a_stem.find_last_of('_');
              size_t b_underscore = b_stem.find_last_of('_');
              if (a_underscore == std::string::npos ||
                  b_underscore == std::string::npos) {
                return a < b;
              }
              int a_frameid = std::stoi(a_stem.substr(0, a_underscore));
              int b_frameid = std::stoi(b_stem.substr(0, b_underscore));
              return a_frameid < b_frameid;
            });

  std::vector<size_t> frameids;
  std::vector<double> timestamps;
  for (const auto &file : geoPoseFiles) {
    std::string _stem = file.stem().string();
    size_t _underscore = _stem.find_last_of('_');
    if (_underscore == std::string::npos) {
      continue;
    }
    size_t _frameid = std::stoul(_stem.substr(0, _underscore));
    double _ts = std::stod(_stem.substr(_underscore + 1, _stem.size()));
    frameids.emplace_back(_frameid);
    timestamps.emplace_back(_ts);
  }

  m_syncFramesInfo.clear();
  for (size_t i = 0; i < frameids.size(); i++) {
    size_t fid = frameids[i];
    double ts = timestamps[i];

    SyncFrameInfo info;
    info.serialnum = fid;

    std::string fidStr = std::to_string(fid);
    std::ostringstream tsStream;
    tsStream << std::fixed << std::setprecision(3) << ts;
    std::string tsStr = tsStream.str();

    info.odom_pose_file = geoPoseFiles[i].string();
    info.timestamp_odom = ts;

    for (size_t camIdx = 0; camIdx < m_cameraNames.size(); camIdx++) {
      std::string imgFile =
          fidStr + "_" + tsStr + "_" + m_cameraNames[camIdx] + ".jpg";
      auto fullImgPath = m_imgFilePath / m_cameraNames[camIdx] / imgFile;
      info.images_file[m_cameraIds[camIdx]] = fullImgPath.string();
      info.timestamps_img[m_cameraIds[camIdx]] = ts;
    }

    for (size_t camIdx = 0; camIdx < m_cameraNames.size(); camIdx++) {
      std::string dymaskFile = fidStr + "_" + tsStr + ".png";
      auto fullDymaskPath = m_maskFilePath / "dynamic" / m_cameraNames[camIdx] / dymaskFile;
      info.images_dymask_file[m_cameraIds[camIdx]] = fullDymaskPath.string();
    }

    for (size_t camIdx = 0; camIdx < m_cameraNames.size(); camIdx++) {
      std::string stmaskFile = "mask.png";
      auto fullStmaskPath = m_maskFilePath / "static" / m_cameraNames[camIdx] / stmaskFile;
      info.images_stmask_file[m_cameraIds[camIdx]] = fullStmaskPath.string();
    }

    info.timestamp_lidar = ts;
    std::string lidarFile = fidStr + "_" + tsStr + ".pcd";
    info.lidar_file = (m_lidarFilePath / lidarFile).string();

    m_syncFramesInfo.emplace_back(info);
  }
  std::cout << "Loaded data index with " << m_syncFramesInfo.size()
            << " frames." << std::endl;
}

fbmv_calib::SyncFrame NonUniformSampling::GetSyncFrame(size_t idx) {
  fbmv_calib::SyncFrame sync_frame;
  auto &info = m_syncFramesInfo[idx];

  sync_frame.serialnum = info.serialnum;
  sync_frame.timestamp_odom = info.timestamp_odom;
  sync_frame.odom_pose = Eigen::Matrix4d::Identity();
  sync_frame.angular_velocity = Eigen::Vector3d::Zero();
  sync_frame.linear_velocity = Eigen::Vector3d::Zero();
  sync_frame.linear_acceleration = Eigen::Vector3d::Zero();
  
  std::lock_guard<std::mutex> lock(this->rgbd_merge_mutex2);
  
  {
    std::string filepath = info.odom_pose_file;
    if (!fs::exists(filepath)) {
      std::cerr << "Error: Pose file does not exist: " << filepath << std::endl;
      std::exit(1);
    }
    YAML::Node yaml = YAML::LoadFile(filepath);
    if (yaml["pose_utm"]) {
      auto pose_utm = yaml["pose_utm"];
      if (pose_utm.IsSequence() && pose_utm.size() == 16) {
        Eigen::Matrix4d T_lidar2local = Eigen::Matrix4d::Identity();
        for (size_t i = 0; i < 16; i++) {
          T_lidar2local(i / 4, i % 4) = pose_utm[i].as<double>();
        }
        sync_frame.odom_pose =
            T_lidar2local * this->m_sensorModel->get_T_vl();
      }
    }
    
    // Read offset_utm for this frame
    if (yaml["offset_utm"]) {
      auto offset_utm = yaml["offset_utm"];
      if (offset_utm.IsSequence() && offset_utm.size() == 3) {
        sync_frame.offset_utm[0] = offset_utm[0].as<double>();
        sync_frame.offset_utm[1] = offset_utm[1].as<double>();
        sync_frame.offset_utm[2] = offset_utm[2].as<double>();
      }
    }
    
    // 线程安全的UTM origin初始化
    {
      if (!m_hasUtmOrigin)
      {
        m_utmOrigin = sync_frame.offset_utm;
        m_hasUtmOrigin = true;
        std::cout << "Set UTM origin to m_utmOrigin: " << m_utmOrigin.transpose() << std::endl;
      }
    }
    sync_frame.offset_utm -= m_utmOrigin;
  }

  for (const auto &[camId, imgFile] : info.images_file) {
    cv::Mat img = cv::imread(imgFile, cv::IMREAD_UNCHANGED);
    if (img.empty()) {
      continue;
    }
    sync_frame.images[camId] = img;
    sync_frame.timestamps_img[camId] = info.timestamps_img.at(camId);

    auto camName = m_cameraNames[camId];
    if (m_sensorModel->camera_params.find(camName) ==
        m_sensorModel->camera_params.end()) {
      continue;
    }
    CameraIntrinsics intr = m_sensorModel->camera_params.at(camName).first;
    fbmv_calib::CameraIntrinsics fbmv_intr;
    fbmv_intr.K = intr.K;
    fbmv_intr.D = cv::Mat(intr.D.size(), 1, CV_64F, intr.D.data()).clone();
    fbmv_intr.model = static_cast<fbmv_calib::CameraModel>(intr.model);
    fbmv_intr.width = intr.width;
    fbmv_intr.height = intr.height;
    fbmv_intr.xi = intr.xi;
    sync_frame.intrinsics[camId] = fbmv_intr;

    Eigen::Matrix4f T_cv =
        m_sensorModel->camera_params.at(camName).second.cast<float>();
    sync_frame.degisn_Tcv[camId] = T_cv;
  }

  return sync_frame;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
NonUniformSampling::GetLidarFrame(size_t idx) {
  if (idx >= m_syncFramesInfo.size()) {
    return nullptr;
  }

  auto &info = m_syncFramesInfo[idx];
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile(info.lidar_file, *lidarCloud) == -1) {
    std::cerr << "Failed to load LiDAR point cloud: " << info.lidar_file << "\n";
    return nullptr;
  }
  // return lidarCloud;

  // 使用PCL CropBox滤波器去除XY在3米以内的点
  pcl::CropBox<pcl::PointXYZI> crop_box_filter;
  crop_box_filter.setInputCloud(lidarCloud);
  // 设置过滤范围：排除[-5,-5,-100]到[5,5,50]的点
  float range_xy = 3.0f;
  crop_box_filter.setMin(Eigen::Vector4f(-range_xy, -range_xy, -5.0, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(range_xy, range_xy, 5.0, 1.0));
  crop_box_filter.setNegative(true); // 只保留范围外的点

  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>());
  crop_box_filter.filter(*filteredCloud);
  
  range_xy = 50.0f;
  crop_box_filter.setMin(Eigen::Vector4f(-range_xy, -range_xy, -5.0, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(range_xy, range_xy, 50.0, 1.0));
  crop_box_filter.setNegative(false); // 只保留范围 内 的点
  crop_box_filter.setInputCloud(filteredCloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud_2(new pcl::PointCloud<pcl::PointXYZI>());
  crop_box_filter.filter(*filteredCloud_2);
  return filteredCloud_2;
}

void NonUniformSampling::RgbExtract(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, size_t idx) {

  const auto &images = GetSyncFrame(idx).images;
  auto cloud_in = GetLidarFrame(idx);
  auto &stmask_files = m_syncFramesInfo[idx].images_stmask_file;
  auto &dymask_files = m_syncFramesInfo[idx].images_dymask_file;
  
  // pcl::io::savePCDFileBinary("/data/exported_roi_data/temp/1.pcd", *cloud_in);
  // std::cout << " 388 original 点云 : " << cloud_in->size() << std::endl;

  // 移除动态物体点云log
  // 目前仅通过mask文件移除动态物体点云
  static int frame_count = 0;
  frame_count++;
  if (1)
  {
    std::string label_file = m_syncFramesInfo[idx].lidar_file;
    // 1. 替换路径中的 pointclouds 为 labels
    size_t pos = label_file.find("/pointclouds/");
    if (pos != std::string::npos) {
      // label_file.replace(pos, 13, "/labels/");
      label_file.replace(pos, 13, "/debug_files/3dbox/");
    }
    // 2. 替换后缀名 .pcd 为 .json
    pos = label_file.rfind(".pcd");
    if (pos != std::string::npos) {
      label_file.replace(pos, 4, "_gd.json");
    }
    if (frame_count % 100 == 0)
    {
      std::cout << "对应p文件: " << m_syncFramesInfo[idx].lidar_file << std::endl;
      std::cout << "对应标签文件: " << label_file << std::endl;
    }
    if (fs::exists(label_file)) {
      std::vector<tydwm::obj::AnnotatedObject> boxes;
      
      const bool only_keep_dynamic = true;

      GetBoxesFromLabelJson(label_file, boxes, only_keep_dynamic);
      // std::cout << "boxes size : " << boxes.size() << std::endl;

      if (!boxes.empty()) {
        // RemovePointcloudDynamicObjects_2d(cloud_in, boxes);
        RemovePointcloudDynamicObjects_3d(cloud_in, boxes);
      }
    }
    else
    {
      std::cout << "对应pcd文件: " << m_syncFramesInfo[idx].lidar_file << std::endl;
      std::cout << "标签文件不存在: 目前通过曼孚标注结果来去除动态物体，需要这个labels文件：" << label_file << std::endl;
      exit(1);
    }

    // std::cout << "  点云移除动态物体后剩余点数: " << cloud_in->size() << std::endl;
    // pcl::io::savePCDFileBinary("/data/exported_roi_data/temp/2.pcd", *cloud_in);
  }

  for (const auto &[camera_id, img] : images) {
    if (img.empty() ||
        img.at<cv::Vec3b>(img.rows / 2, img.cols / 2) == cv::Vec3b(0, 0, 0)) {
      continue;
    }

    std::string camera_name = m_cameraNames[camera_id];
    fs::path stmask_file = stmask_files.at(camera_id);
    fs::path dymask_file = dymask_files.at(camera_id);
    // std::cout << "stmask_files: " << stmask_file << std::endl;
    // std::cout << "dymask_files: " << dymask_file << std::endl;

    cv::Mat img_mask;
    {
      auto dkernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
      cv::Mat dyMask, stMask;
      if (fs::exists(stmask_file)) {
        stMask = cv::imread(stmask_file.string(), cv::IMREAD_GRAYSCALE);
        if (!stMask.empty()) {
          cv::resize(stMask, stMask, cv::Size(img.cols, img.rows), 0, 0,
                     cv::INTER_NEAREST);
          if (stMask.type() != CV_8UC1) {
            cv::cvtColor(stMask, stMask, cv::COLOR_BGR2GRAY);
          }
          cv::dilate(stMask, stMask, dkernel);
        }
      }
      if (fs::exists(dymask_file)) {
        dyMask = cv::imread(dymask_file.string(), cv::IMREAD_GRAYSCALE);
        if (!dyMask.empty()) {
          cv::resize(dyMask, dyMask, cv::Size(img.cols, img.rows), 0, 0,
                     cv::INTER_NEAREST);
          if (dyMask.type() != CV_8UC1) {
            cv::cvtColor(dyMask, dyMask, cv::COLOR_BGR2GRAY);
          }
          cv::dilate(dyMask, dyMask, dkernel);
        }
      }

      if (!stMask.empty() && !dyMask.empty()) {
        img_mask = stMask & dyMask;
      } else if (!stMask.empty()) {
        img_mask = stMask;
      } else if (!dyMask.empty()) {
        img_mask = dyMask;
      }
    }

    const auto &intrinsic =
        this->m_sensorModel->camera_params[camera_name].first;
    const Eigen::Matrix4d &T_cv =
        this->m_sensorModel->camera_params[camera_name].second;
    Eigen::Matrix4d T_lc = T_cv.inverse() * this->m_sensorModel->get_T_lv();

    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    pcl::transformPointCloud(*cloud_in, transformed_cloud, T_lc);
    std::vector<size_t> keep_idxs;

    for (int i = 0; i < transformed_cloud.points.size(); i++) {
      cv::Point3f pc(transformed_cloud.points[i].x,
                     transformed_cloud.points[i].y,
                     transformed_cloud.points[i].z);

      std::vector<cv::Point2f> pi_;
      if (pc.z <= 0) {
        keep_idxs.emplace_back(i);
        continue;
      }

      if (intrinsic.model == CameraModel::Brown &&
          intrinsic.type == CameraType::Pinhole) {
        cv::projectPoints(
            std::vector<cv::Point3f>{pc}, cv::Mat::zeros(3, 1, CV_64F),
            cv::Mat::zeros(3, 1, CV_64F), intrinsic.K, intrinsic.D, pi_);
      } else {
        keep_idxs.emplace_back(i);
        continue;
      }

      double u = pi_[0].x;
      double v = pi_[0].y;

      int u_ = std::round(u);
      int v_ = std::round(v);
      if (u_ < 0 || u_ >= img.cols || v_ < 0 || v_ >= img.rows) {
        keep_idxs.emplace_back(i);
        continue;
      }

      if (!img_mask.empty()) {
        if (img_mask.at<uchar>(v_, u_) < 128) {
          continue;
        }
      }

      cv::Vec3b org_col = img.at<cv::Vec3b>(v_, u_);

      pcl::PointXYZRGB point;
      point.x = cloud_in->points[i].x;
      point.y = cloud_in->points[i].y;
      point.z = cloud_in->points[i].z;
      point.r = org_col[2];
      point.g = org_col[1];
      point.b = org_col[0];
      if (point.r == 255 && point.g == 255 && point.b == 255) {
        continue;
      }
      cloud_out->points.emplace_back(point);
    }

    if (keep_idxs.empty() || cloud_in->empty()) {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI> new_cloud;
    new_cloud.points.reserve(keep_idxs.size());
    for (const auto &idx : keep_idxs) {
      new_cloud.points.emplace_back(cloud_in->points[idx]);
    }
    new_cloud.width = new_cloud.points.size();
    new_cloud.height = 1;
    new_cloud.is_dense = true;
    *cloud_in = new_cloud;
  }

  cloud_out->width = cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = true;
  
  // pcl::io::savePCDFileBinary("/data/exported_roi_data/temp/3.pcd", *cloud_out);
  // std::cout << "509 --------exit can---------- Extracted rgbd points: " << cloud_out->size() << std::endl;
  // exit(1);
}

template <typename PointT>
void NonUniformSampling::MapBlock(typename pcl::PointCloud<PointT>::Ptr src,
                                  float grid_sz, float leaf_sz,
                                  size_t min_points_threshold) {

  if (src->empty()) {
    return;
  }

  m_rgbdBlockInfos.clear();

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::min();

  for (const auto &point : src->points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  int grid_x = static_cast<int>(std::ceil((max_x - min_x) / grid_sz));
  int grid_y = static_cast<int>(std::ceil((max_y - min_y) / grid_sz));

  std::vector<std::vector<std::vector<int>>> grid_indices(
      grid_x, std::vector<std::vector<int>>(grid_y));

  for (int i = 0; i < src->points.size(); ++i) {
    const auto &point = src->points[i];
    int grid_i =
        std::min(static_cast<int>((point.x - min_x) / grid_sz), grid_x - 1);
    int grid_j =
        std::min(static_cast<int>((point.y - min_y) / grid_sz), grid_y - 1);
    grid_indices[grid_i][grid_j].push_back(i);
  }

  for (int i = 0; i < grid_x; ++i) {
    for (int j = 0; j < grid_y; ++j) {
      if (grid_indices[i][j].size() > min_points_threshold) {
        typename pcl::PointCloud<PointT>::Ptr block_cloud(
            new pcl::PointCloud<PointT>);

        float block_min_x = min_x + i * grid_sz;
        float block_max_x = min_x + (i + 1) * grid_sz;
        float block_min_y = min_y + j * grid_sz;
        float block_max_y = min_y + (j + 1) * grid_sz;

        for (int idx : grid_indices[i][j]) {
          block_cloud->points.push_back(src->points[idx]);
        }

        block_cloud->width = block_cloud->points.size();
        block_cloud->height = 1;
        block_cloud->is_dense = false;

        typename pcl::PointCloud<PointT>::Ptr filtered_cloud(
            new pcl::PointCloud<PointT>());

        if (leaf_sz > 0) {
          pcl::VoxelGrid<PointT> voxel_filter;
          voxel_filter.setInputCloud(block_cloud);
          voxel_filter.setLeafSize(leaf_sz, leaf_sz, leaf_sz);
          voxel_filter.filter(*filtered_cloud);
        } else {
          filtered_cloud = block_cloud;
        }

        fs::path submap_path =
            ("submap_" + std::to_string(i) + "_" + std::to_string(j) + ".pcd");
        // pcl::io::savePCDFileBinaryCompressed(
        //     m_outputRgbdPath / submap_path.string(), *filtered_cloud);

        RgbdBlockInfo block_info;
        block_info.file_path = submap_path.string();
        block_info.grid_x = i;
        block_info.grid_y = j;
        block_info.point_count = filtered_cloud->points.size();
        block_info.bounding_box.min_x = block_min_x;
        block_info.bounding_box.max_x = block_max_x;
        block_info.bounding_box.min_y = block_min_y;
        block_info.bounding_box.max_y = block_max_y;

        m_rgbdBlockInfos.push_back(block_info);
      }
    }
  }

  using namespace rapidjson;
  Document document;
  document.SetObject();
  Document::AllocatorType &allocator = document.GetAllocator();

  document.AddMember("total_blocks", static_cast<int>(m_rgbdBlockInfos.size()),
                     allocator);

  Value blocks(kArrayType);
  for (const auto &block_info : m_rgbdBlockInfos) {
    Value block(kObjectType);
    Value file_path_val;
    file_path_val.SetString(block_info.file_path.c_str(),
                            block_info.file_path.length(), allocator);
    block.AddMember("file_path", file_path_val, allocator);
    block.AddMember("grid_x", block_info.grid_x, allocator);
    block.AddMember("grid_y", block_info.grid_y, allocator);
    block.AddMember("point_count", static_cast<int>(block_info.point_count),
                    allocator);

    Value bounding_box(kObjectType);
    bounding_box.AddMember("min_x", block_info.bounding_box.min_x, allocator);
    bounding_box.AddMember("max_x", block_info.bounding_box.max_x, allocator);
    bounding_box.AddMember("min_y", block_info.bounding_box.min_y, allocator);
    bounding_box.AddMember("max_y", block_info.bounding_box.max_y, allocator);
    block.AddMember("bounding_box", bounding_box, allocator);
    blocks.PushBack(block, allocator);
  }

  document.AddMember("blocks", blocks, allocator);

  FILE *fp =
      fopen((m_outputRgbdPath / "rgbd_block_info.json").string().c_str(), "wb");
  if (fp) {
    char writeBuffer[65536];
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    PrettyWriter<FileWriteStream> writer(os);
    document.Accept(writer);
    fclose(fp);
  }
}

bool NonUniformSampling::LoadRgbdBlockInfo() {
  using namespace rapidjson;

  m_rgbdBlockInfos.clear();

  fs::path json_path = m_outputRgbdPath / "rgbd_block_info.json";

  if (!fs::exists(json_path)) {
    std::cerr << "RGBD block info file does not exist: " << json_path << std::endl;
    return false;
  }

  FILE *fp = fopen(json_path.string().c_str(), "rb");
  if (!fp) {
    return false;
  }

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  Document document;
  document.ParseStream(is);
  fclose(fp);

  if (document.HasParseError() || !document.IsObject() ||
      !document.HasMember("blocks") || !document["blocks"].IsArray()) {
    return false;
  }

  const Value &blocks = document["blocks"];
  for (SizeType i = 0; i < blocks.Size(); i++) {
    const Value &block = blocks[i];

    RgbdBlockInfo block_info;
    block_info.file_path = block["file_path"].GetString();
    block_info.grid_x = block["grid_x"].GetInt();
    block_info.grid_y = block["grid_y"].GetInt();
    block_info.point_count = block["point_count"].GetInt();

    const Value &bbox = block["bounding_box"];
    block_info.bounding_box.min_x = bbox["min_x"].GetFloat();
    block_info.bounding_box.max_x = bbox["max_x"].GetFloat();
    block_info.bounding_box.min_y = bbox["min_y"].GetFloat();
    block_info.bounding_box.max_y = bbox["max_y"].GetFloat();

    m_rgbdBlockInfos.push_back(block_info);
  }

  std::cout << "Successfully loaded " << m_rgbdBlockInfos.size()
            << " RGBD block info records" << std::endl;

  return !m_rgbdBlockInfos.empty();
}

template void NonUniformSampling::MapBlock<pcl::PointXYZRGB>(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, float grid_sz, float leaf_sz,
    size_t min_points_threshold);

bool NonUniformSampling::ReadUtmOrigin(Eigen::Vector3f &origin_utm) {
  if (m_syncFramesInfo.empty()) {
    return false;
  }
  
  if (!m_hasUtmOrigin) {
    std::cerr << "UTM origin not initialized. Please run RunRgbdMerge first." << std::endl;
    return false;
  }
  
  origin_utm = m_utmOrigin.cast<float>();
  std::cout << "Using UTM origin: " << origin_utm.transpose() << std::endl;
  
  return true;
}

std::vector<Triangle> NonUniformSampling::Delaunay(
    const std::vector<cv::Point2f> &points, const cv::Rect2f &rect,
    float maxEdgeLength) {
  std::vector<Triangle> triangleIndices;

  if (points.size() < 3) {
    return triangleIndices;
  }

  // 浮点数比较的容差（处理精度问题）
  const double EPS = 1e-6;

  // 内部lambda：判断两个点是否近似相等
  auto isPointEqual = [&EPS](const cv::Point2f &a, const cv::Point2f &b) {
    return fabs(a.x - b.x) < EPS && fabs(a.y - b.y) < EPS;
  };

  // 内部lambda：查找点在原始点集中的索引
  auto findPointIndex = [&points, &isPointEqual](const cv::Point2f &p) {
    for (size_t i = 0; i < points.size(); ++i) {
      if (isPointEqual(p, points[i])) {
        return static_cast<int>(i);
      }
    }
    return -1; // 未找到（可能是边界矩形的虚拟点）
  };

  // 内部lambda：计算两点间距离的平方（避免开方，提高效率）
  auto distanceSq = [](const cv::Point2f &a, const cv::Point2f &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
  };

  // 内部lambda：检测三点是否共线（使用叉积判断）
  auto areCollinear = [&EPS](const cv::Point2f &p1, const cv::Point2f &p2,
                             const cv::Point2f &p3) {
    // 计算向量 p1->p2 和 p1->p3 的叉积
    double v1x = p2.x - p1.x;
    double v1y = p2.y - p1.y;
    double v2x = p3.x - p1.x;
    double v2y = p3.y - p1.y;

    // 叉积的绝对值（表示平行四边形面积，三角形面积的2倍）
    double crossProduct = fabs(v1x * v2y - v1y * v2x);

    // 如果叉积接近0，说明三点共线
    return crossProduct < EPS;
  };

  // 1. 初始化剖分器
  cv::Subdiv2D subdiv(rect);

  // 2. 插入所有点
  for (const auto &p : points) {
    subdiv.insert(p);
  }

  // 3. 提取三角形坐标
  std::vector<cv::Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);

  // 4. 转换为原始点集的索引
  const double maxEdgeSq =
      maxEdgeLength * maxEdgeLength; // 阈值的平方（避免重复开方）
  int collinear_filtered = 0;        // 统计过滤的共线三角形数量

  for (const auto &t : triangleList) {
    // 解析三角形的三个顶点坐标
    cv::Point2f p1(t[0], t[1]);
    cv::Point2f p2(t[2], t[3]);
    cv::Point2f p3(t[4], t[5]);

    // 查找每个顶点在原始点集中的索引
    int idx1 = findPointIndex(p1);
    int idx2 = findPointIndex(p2);
    int idx3 = findPointIndex(p3);

    // 仅保留三个顶点都在原始点集中的三角形
    if (idx1 == -1 || idx2 == -1 || idx3 == -1) {
      continue; // 跳过含虚拟点的三角形
    }

    // 检查三点是否共线
    if (areCollinear(points[idx1], points[idx2], points[idx3])) {
      collinear_filtered++;
      continue; // 跳过共线的三角形
    }

    // 计算三边长度的平方
    double d12Sq = distanceSq(points[idx1], points[idx2]);
    double d23Sq = distanceSq(points[idx2], points[idx3]);
    double d31Sq = distanceSq(points[idx3], points[idx1]);

    // 找到最长边的平方
    double maxD_Sq = std::max({d12Sq, d23Sq, d31Sq});

    // 最长边不超过阈值时保留该三角形
    if (maxD_Sq <= maxEdgeSq + EPS) { // 容差处理
      triangleIndices.push_back({idx1, idx2, idx3});
    }
  }

  // 输出统计信息
  if (collinear_filtered > 0) {
    std::cout << "Delaunay三角剖分: 过滤了 " << collinear_filtered
              << " 个共线三角形" << std::endl;
  }
  std::cout << "Delaunay三角剖分: 从 " << points.size() << " 个点生成了 "
            << triangleIndices.size() << " 个有效三角形" << std::endl;

  return triangleIndices;
}

void NonUniformSampling::SaveMeshToPLY(
    const std::vector<Eigen::Vector3f> &cloud,
    const std::vector<Triangle> &triangles, const std::string &filename) {
  std::ofstream fout(m_outputPath / filename);
  if (!fout.is_open()) {
    std::cerr << "无法打开文件：" << filename << std::endl;
    return;
  }

  // PLY文件头
  fout << "ply\n";
  fout << "format ascii 1.0\n";
  fout << "element vertex " << cloud.size() << "\n";
  fout << "property float x\n";
  fout << "property float y\n";
  fout << "property float z\n";
  fout << "element face " << triangles.size() << "\n";
  fout << "property list uchar int vertex_indices\n";
  fout << "end_header\n";

  // 写入顶点
  for (const auto &p : cloud) {
    fout << p[0] << " " << p[1] << " " << p[2] << "\n";
  }

  // 写入三角形（每个面3个顶点索引）
  for (const auto &t : triangles) {
    fout << "3 " << t.v0 << " " << t.v1 << " " << t.v2 << "\n";
  }

  fout.close();
  std::cout << "Mesh已保存到：" << (m_outputPath / filename).string()
            << "（顶点数：" << cloud.size() << "，三角形数：" << triangles.size()
            << "）" << std::endl;
}

bool NonUniformSampling::IsPointInBox(
    const Eigen::Vector3f &pt, const tydwm::obj::AnnotatedObject &box) {
  // 世界坐标下，变换到box的局部坐标系，判断是否在[-x/2,x/2]等范围内
  Eigen::Matrix4f inv = box.getWorldTransform().inverse();
  Eigen::Vector4f pt_h(pt.x(), pt.y(), pt.z(), 1.0f);
  Eigen::Vector4f local = inv * pt_h;
  float hx = (0.4 + box.geometry.size.x) * 0.5f;
  float hy = (0.4 + box.geometry.size.y) * 0.5f;
  float hz = box.geometry.size.z * 0.5f;
  return (local.x() >= -hx && local.x() <= hx &&
          local.y() >= -hy && local.y() <= hy &&
          local.z() >= -hz && local.z() <= hz);
}

void NonUniformSampling::GetBoxesFromLabelJson(
    const std::string &label_json,
    std::vector<tydwm::obj::AnnotatedObject> &out_boxes,
    bool only_keep_dynamic) {
  
  // 检查标签文件是否存在
  if (!fs::exists(label_json)) {
    std::cerr << "[警告] 标签文件不存在: " << label_json << std::endl;
    return;
  }
  
  // 加载标签文件
  tydwm::obj::ObjectAnnotationLoader loader;
  if (!loader.LoadFromFile(label_json)) {
    std::cerr << "[错误] 无法加载标签文件: " << label_json << std::endl;
    return;
  }
  
  // 收集所有box对象
  for (const auto &obj : loader.GetSceneAnnotation().objects) {
    if (only_keep_dynamic) {
      if (  obj.labelName == "动态目标"  ) // 仅保留动态物体
      {
        out_boxes.push_back(obj);
      }
    } else {
      out_boxes.push_back(obj);
    }
  }
}

// 基于box和点云都投影到2D，再移除动态物体点云 ，速度略快于3d版
// 测试发现部分地面点会被误移除
void NonUniformSampling::RemovePointcloudDynamicObjects_2d(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const std::vector<tydwm::obj::AnnotatedObject> &boxes) {
  
  if (boxes.empty() || !cloud || cloud->points.empty()) {
    return;
  }
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  filtered_cloud->header = cloud->header;
  filtered_cloud->is_dense = cloud->is_dense;
  
  int removed_count = 0;
  std::vector<int> keep_indices;
  keep_indices.reserve(cloud->points.size());

  // 预处理所有box的2D投影参数
  struct Box2D {
    Eigen::Vector2f center;
    float yaw;
    float half_x;
    float half_y;
    float z_base;
  };
  std::vector<Box2D> boxes2d;
  boxes2d.reserve(boxes.size());
  for (const auto &box : boxes) {
    Eigen::Vector3f c3d = box.getWorldTransform().block<3,1>(0,3);
    float yaw = 0.0f;
    const Eigen::Matrix3f rot = box.getWorldTransform().block<3,3>(0,0);
    yaw = std::atan2(rot(1,0), rot(0,0));
    float hx = box.geometry.size.x * 0.5f;
    float hy = box.geometry.size.y * 0.5f;
    float z_base = c3d.z() - box.geometry.size.z * 0.5f;
    boxes2d.push_back({c3d.head<2>(), yaw, hx, hy, z_base});
  }

  #pragma omp parallel for schedule(static)
  for (int i = 0; i < cloud->points.size(); ++i) {
    const auto &pt = cloud->points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    Eigen::Vector2f point2d(pt.x, pt.y);
    bool inside_box = false;
    for (size_t b = 0; b < boxes2d.size(); ++b) {
      const auto &box2d = boxes2d[b];
      // 坐标系变换：点减去box中心，旋转到box局部坐标
      Eigen::Vector2f rel = point2d - box2d.center;
      float cos_yaw = std::cos(-box2d.yaw);
      float sin_yaw = std::sin(-box2d.yaw);
      float x_local = rel.x() * cos_yaw - rel.y() * sin_yaw;
      float y_local = rel.x() * sin_yaw + rel.y() * cos_yaw;
      // 只移除z高于box底面的点
      if (pt.z > box2d.z_base &&
          x_local >= -box2d.half_x && x_local <= box2d.half_x &&
          y_local >= -box2d.half_y && y_local <= box2d.half_y) {
        inside_box = true;
        #pragma omp atomic
        removed_count++;
        break;
      }
    }
    if (!inside_box) {
      #pragma omp critical
      keep_indices.push_back(i);
    }
  }
  filtered_cloud->points.reserve(keep_indices.size());
  for (const auto &idx : keep_indices) {
    filtered_cloud->points.push_back(cloud->points[idx]);
  }
  
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  
  // if (removed_count > 0) {
  //   std::cout << "  [动态物体移除] 移除 " << removed_count << " 个点, 保留 "
  //             << filtered_cloud->points.size() << " 个点" << std::endl;
  // }
  
  // 更新原始点云
  *cloud = *filtered_cloud;
}


void NonUniformSampling::RemovePointcloudDynamicObjects_3d(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const std::vector<tydwm::obj::AnnotatedObject> &boxes) {
  
  if (boxes.empty() || !cloud || cloud->points.empty()) {
    return;
  }
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  filtered_cloud->header = cloud->header;
  filtered_cloud->is_dense = cloud->is_dense;
  
  int removed_count = 0;
  std::vector<int> keep_indices;
  keep_indices.reserve(cloud->points.size());
  #pragma omp parallel for schedule(static)
  for (int i = 0; i < cloud->points.size(); ++i) {
    const auto &pt = cloud->points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    Eigen::Vector3f point(pt.x, pt.y, pt.z);
    bool inside_box = false;
    for (const auto &box : boxes) {
      if (IsPointInBox(point, box)) {
        inside_box = true;
        #pragma omp atomic
        removed_count++;
        break;
      }
    }
    if (!inside_box) {
      #pragma omp critical
      keep_indices.push_back(i);
    }
  }
  filtered_cloud->points.reserve(keep_indices.size());
  for (const auto &idx : keep_indices) {
    filtered_cloud->points.push_back(cloud->points[idx]);
  }
  
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  
  // if (removed_count > 0) {
  //   std::cout << "  [动态物体移除] 移除 " << removed_count << " 个点, 保留 "
  //             << filtered_cloud->points.size() << " 个点" << std::endl;
  // }
  
  // 更新原始点云
  *cloud = *filtered_cloud;
}
