#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <vector>

int main(int argc, char** argv) 
{
    std::string filename = "/mnt/nvme0n1p2/data/1011/dense_global_map_xyzi.pcd";
    int grid_x_num = 3; // 默认X方向2个网格
    int grid_y_num = 3; // 默认Y方向2个网格

    if (argc >= 2) {
        filename = argv[1];
    }
    if (argc >= 3) {
        grid_x_num = std::stoi(argv[2]);
    }
    if (argc >= 4) {
        grid_y_num = std::stoi(argv[3]);
    }

    // 提取最后一个/之前的所有字符（即目录路径）
    std::string dir_path = "";
    size_t last_slash = filename.find_last_of('/');
    if (last_slash != std::string::npos) {
        dir_path = filename.substr(0, last_slash);
    }

    std::cout << "filename: " << filename << std::endl;
    std::cout << "directory path: " << dir_path << std::endl;
    std::cout << "Grid size: " << grid_x_num << "x" << grid_y_num << " = " << grid_x_num * grid_y_num << " grids" << std::endl;
    
    // mc cp /mnt/nvme0n1p2/data/0930/build/hesai_0930/ minio-rsu/tyjt-rsu/qcsl_map/map   --recursive

    Eigen::Affine3d T_wl = Eigen::Affine3d::Identity();
    // offset_utm

    Eigen::Vector3d offset_utm(274534.52320612938 , 3479025.2756054322 , 13.507352803009168  - ( 0 ) );

    Eigen::Vector3d suzhou_original_utm( 275000.0223369 , 3479281.54229995, 0.0 );

    T_wl.translation() =  offset_utm - suzhou_original_utm;

    Eigen::Quaterniond q1( 1.0, 0.0, 0.0, 0.0  );
    T_wl.rotate(q1);
    std::cout << "T_wl: " << T_wl.matrix()  << std::endl << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud_o) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "cloud_o->size(): " << cloud_o->size() << std::endl;

    // 对点云进行变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_o, *transformedCloud, T_wl);
    
    std::string output_filename = dir_path + "/1011_suzhou_x1_6.pcd";
    pcl::io::savePCDFileBinary(output_filename, *transformedCloud);
    std::cout << "transformedCloud->size(): " << transformedCloud->size() << std::endl;
    std::cout << "output_filename: " << output_filename << std::endl;

    // return -1;

    if (grid_y_num  * grid_x_num == 1 )
    {
        return -1;
    }

    // 计算边界框
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*transformedCloud, minPt, maxPt);
    
    float x_range = maxPt.x - minPt.x;
    float y_range = maxPt.y - minPt.y;
    float grid_x_size = x_range / grid_x_num;
    float grid_y_size = y_range / grid_y_num;
    
    int total_grids = grid_x_num * grid_y_num;
    std::cout << "X range: " << x_range << ", Y range: " << y_range << std::endl;
    std::cout << "Grid size: " << grid_x_size << " x " << grid_y_size << std::endl;

    // 创建网格点云容器
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> gridClouds(total_grids);
    for (int i = 0; i < total_grids; i++) {
        gridClouds[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    // 将点分配到网格
    for (const auto& point : transformedCloud->points) {
        int grid_x_idx = std::min(grid_x_num - 1, (int)((point.x - minPt.x) / grid_x_size));
        int grid_y_idx = std::min(grid_y_num - 1, (int)((point.y - minPt.y) / grid_y_size));
        int grid_idx = grid_y_idx * grid_x_num + grid_x_idx;
        
        if (grid_idx >= 0 && grid_idx < total_grids) {
            gridClouds[grid_idx]->points.push_back(point);
        }
    }

    // 保存网格文件
    for (int i = 0; i < total_grids; i++) {
        if (!gridClouds[i]->points.empty()) {
            gridClouds[i]->width = gridClouds[i]->size();
            gridClouds[i]->height = 1;
            output_filename = dir_path +"/suzhou_qcsl_0930_grid_" + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(output_filename, *gridClouds[i]);
            std::cout << "Saved grid " << i << " with " << gridClouds[i]->size() << " points to " << output_filename << std::endl;
        }
    }

    return 0;
}
