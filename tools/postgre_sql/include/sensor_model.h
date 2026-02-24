#pragma once

#include "common.h"
#include <Eigen/Eigen>
#include <filesystem>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>

namespace fs = std::filesystem;

// xV + 1L+ 1i
class sensor_model {
private:
    bool            is_Tli_hotfix;
    fs::path        base_path;
    fs::path        extrinsics_path;
    fs::path        intrinsics_path;
    Eigen::Matrix4d T_lv, T_vl;
    Eigen::Matrix4d T_iv_init, T_vi_init;
    Eigen::Matrix4d T_li_init, T_il_init;

    Eigen::Matrix4d T_iv_hotfix, T_vi_hotfix;
    Eigen::Matrix4d T_li_hotfix, T_il_hotfix;

    int load_camera_intrinsics(std::string yaml_file, CameraIntrinsics &internal);

public:
    explicit sensor_model(std::string calib_folder, std::string extrinsics_folder, std::string intrinsics_folder);
    ~sensor_model();

    std::map<std::string, std::pair<CameraIntrinsics, Eigen::Matrix4d>> camera_params;
    std::map<std::string, std::string>                                  camera_intrinsics_files;
    std::map<std::string, CameraIntrinsics>                             cropped_intrinsics;

    bool load_all();
    void crop_all_cameras(double cropx, double cropy);
    void Save_Internal_Matrix(std::string filename, CameraIntrinsics &intrinsics);

    Eigen::Matrix4d get_T_lv();
    Eigen::Matrix4d get_T_iv();
    Eigen::Matrix4d get_T_li();
    Eigen::Matrix4d get_T_vl();
    Eigen::Matrix4d get_T_vi();
    Eigen::Matrix4d get_T_il();

    void Hotfix_T_li(const Eigen::Matrix4d &T_hotfix);
};
