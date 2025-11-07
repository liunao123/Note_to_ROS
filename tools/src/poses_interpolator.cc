/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <iostream>
#include <string>

#include "poses_interpolation_standalone.h"

int main(int argc, char **argv) {
  if (argc < 4 || argc > 5) {
    std::cout << "Usage: " << argv[0] 
              << " <input_poses_path> <ref_timestamps_path> <output_poses_path> [extrinsic_path]" 
              << std::endl;
    return 0;
  }

  std::string input_poses_path = argv[1];
  std::string ref_timestamps_path = argv[2];
  std::string out_poses_path = argv[3];
  std::string extrinsic_path = "";
  
  // 如果参数有4个，则设置extrinsic_path
  if (argc == 5) {
    extrinsic_path = argv[4];
  }

  apollo::localization::msf::PosesInterpolation pose_interpolation;
  bool success = pose_interpolation.Init(input_poses_path, ref_timestamps_path,
                                         out_poses_path, extrinsic_path);

  if (success) {
    std::cout << "Starting pose interpolation..." << std::endl;
    pose_interpolation.DoInterpolation();
    std::cout << "Pose interpolation completed!" << std::endl;
  } else {
    std::cerr << "Failed to initialize pose interpolation!" << std::endl;
    return 1;
  }

  return 0;
}
