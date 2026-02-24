#include "NonUniformSampling.h"
#include "sensor_model.h"
#include "utils.h"
#include <getopt.h>

int main(int argc, char *argv[]) {

  fs::path input_path = "/data/exported_roi_data/resorted_data/";

  bool enable_rgbd_merge = true;
  bool enable_non_uniform_sampling = true;

  static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"input", required_argument, 0, 'i'},
      {"rgbd_merge", no_argument, 0, 'r'},
      {"nu_sample", no_argument, 0, 'n'},
      {0, 0, 0, 0}};

  int opt;
  int option_index = 0;
  while ((opt = getopt_long(argc, argv, "hi:rn", long_options,
                            &option_index)) != -1) {
    switch (opt) {
    case 'h':
      std::cout << "Usage: " << argv[0] << " [options]\n";
      std::cout << "Options:\n";
      std::cout << "  -i, --input <path>       Input data folder path\n";
      std::cout << "  -r, --rgbd_merge         Enable RGBD merge\n";
      std::cout << "  -n, --nu_sample          Enable non-uniform sampling\n";
      std::cout << "  -h, --help               Show this help message\n";
      return 0;
    case 'i':
      input_path = optarg;
      break;
    case 'r':
      enable_rgbd_merge = true;
      break;
    case 'n':
      enable_non_uniform_sampling = true;
      break;
    default:
      std::cerr << "Unknown option\n";
      return 1;
    }
  }

  fs::path output_folder = input_path / "sparse";
  fs::path calib_folder = input_path / "calib";
  fs::path images_folder = input_path / "images";
  fs::path masks_folder = input_path / "masks";
  fs::path clouds_folder = input_path / "pointclouds";
  // fs::path labels_folder = input_path / "labels"; // not used here 加载的时候通过其他文件去构造
  fs::path hdmapFolder = input_path / "hd_map";
  if (!fs::exists(hdmapFolder))
  {
    std::cerr << "HDMap folder does not exist: " << hdmapFolder << std::endl;
    return 1;
  }
  std::cout << "HDMap loaded from: " << hdmapFolder << std::endl;

  std::cout << "========================================\n";
  std::cout << "  Non-Uniform Sampling Program\n";
  std::cout << "========================================\n";
  std::cout << "Input folder: " << input_path << "\n";
  std::cout << "Output folder: " << output_folder << "\n";
  std::cout << "Enabled functions:\n";
  std::cout << "  RGBD merge: " << (enable_rgbd_merge ? "ON" : "OFF") << "\n";
  std::cout << "  Non-uniform sampling: " << (enable_non_uniform_sampling ? "ON" : "OFF") << "\n";
  std::cout << "========================================\n";

  std::shared_ptr<sensor_model> sensor_model_ptr =
      std::make_shared<sensor_model>(calib_folder, "extrinsics",
                                     "new_intrinsics");
  if (!sensor_model_ptr->load_all()) {
    std::cerr << "Failed to load sensor model from folder: " << calib_folder << "\n";
    return 1;
  }
  std::cout << "Loaded sensor model successfully.\n";

  std::unique_ptr<NonUniformSampling> sampling_ptr =
      std::make_unique<NonUniformSampling>();
  sampling_ptr->SetSensorModel(sensor_model_ptr);
  sampling_ptr->SetPaths(output_folder / "vehicle_geo_pose", images_folder,
                        clouds_folder, masks_folder, hdmapFolder);
  sampling_ptr->LoadDataIndex();

  if (enable_rgbd_merge) {
    std::cout << "\nRunning RGBD merge..." << std::endl;
    sampling_ptr->RunRgbdMerge();
  }

  if (enable_non_uniform_sampling) {
    std::cout << "\nRunning non-uniform sampling..." << std::endl;
    sampling_ptr->RunNonUniformSampling();
  }

  std::cout << "\nProgram completed successfully." << std::endl;
  return 0;
}
