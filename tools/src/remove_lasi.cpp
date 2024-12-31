#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/types.h>
#include <dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZI> pcxyz_type;

float DISTANCE_THRESHOLD = 0.1;
int PTS_COUNT_THRESHOLD = 5;

void scan_dir_get_filename(string path, vector<string> &filenames)
{
    struct dirent **entry_list;
    int count;
    int i;

    count = scandir(path.c_str(), &entry_list, 0, alphasort);
    if (count < 0)
    {
        perror("scandir");
    }

    for (i = 0; i < count; i++)
    {
        struct dirent *entry;
        entry = entry_list[i];
        // printf("%s\n", entry->d_name);
        // 跳过 ./ 和 ../ 两个目录
        if (i < 2)
        {
            continue;
        }
        filenames.push_back(path + std::string(entry->d_name));
        // std::cout << "filenames  is : " << filenames.back() << std::endl;

        free(entry);
    }
    std::cout << "filenames size is : " << filenames.size() << std::endl;

    free(entry_list);
}

void read_pcd_file(const std::string filename, pcxyz_type::Ptr &cloud)
{
    try
    {
        pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud);
    }
    catch (const std::exception &e)
    {
        PCL_ERROR("can not load pcd file! :  %s", filename.c_str());
        std::cerr << e.what() << '\n';
    }
}

/**
 * line : 过原点的直线
 * pts : 到这个直线的距离
 */
const float get_distance_to_line(const Eigen::Vector3f line, const Eigen::Vector3f pts)
{
    Eigen::Vector3f original(0, 0, 0);
    auto OA = line - original;

    return ((pts - original).cross(pts - line)).norm() / (line - original).norm();

    //     d = |(P0 - P1) x (P0 - P2)| / |P2 - P1|
    // 其中：

    // P0 = (x0, y0, z0) 是点的坐标。
    // P1 = (x1, y1, z1) 和 P2 = (x2, y2, z2) 是直线经过的两点。
    // |v| 表示向量 v 的模长。
    // x 表示向量的叉乘。
}

const pcl::PointCloud<pcl::PointXYZI>::Ptr get_inflation_points(const std::string pcd_file)
{
    // 获取开始时间
    auto start = std::chrono::high_resolution_clock::now();

    pcxyz_type::Ptr cloud(new pcxyz_type);
    pcxyz_type::Ptr cloud_inflat(new pcxyz_type);
    pcxyz_type::Ptr cloud_normal(new pcxyz_type);

    read_pcd_file(pcd_file, cloud);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        const auto &search_point = cloud->points[i];
        // if (search_point.intensity < 90.0 || search_point.x > 15.0 )
        //     continue;

        // 点到直线的距离
        Eigen::Vector3f line(search_point.x, search_point.y, search_point.z);

        int near_pts_count = 0;
        std::vector<float> pts_x;
        float mean_intensity = 0;
        for (size_t j = 0; j < cloud->points.size(); j++)
        {
            const Eigen::Vector3f one_point(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
            // if (cloud->points[j].intensity < 90.0 || cloud->points[j].x > 15.0 )
            //     continue;

            const auto distance = get_distance_to_line(line, one_point);
            if (distance < DISTANCE_THRESHOLD )
            {
                near_pts_count++;
                pts_x.emplace_back( one_point.norm() );
                mean_intensity += cloud->points[j].intensity;
                // cout << cloud->points[j].x  << "  ";
            }
        }

        if ( near_pts_count > PTS_COUNT_THRESHOLD )
        {
            Eigen::VectorXf vx = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(pts_x.data(), pts_x.size());
            double mean = vx.mean();                               // 计算均值
            double vx_variance = ( vx.array() -  mean  ).square().mean() / vx.size(); // 计算方差
            // cout << mean_intensity / pts_x.size() << "  " << endl;

            const double variance_THRESH = 0.08 ;
            if (  vx_variance > variance_THRESH * variance_THRESH )   // x 方向的距离方差阈值
            {
                // if (mean_intensity / pts_x.size() > 50.0) // 平均强度
                if ( 1 ) // 平均强度
                {
                    cloud_inflat->points.emplace_back(search_point);
                }
            }
        }
        else
        {
            cloud_normal->points.emplace_back(search_point);
        }
    }
    cout << "inflation cloud points size : " << cloud_inflat->size() << endl;
    cout << "cloud_normal cloud points size : " << cloud_normal->size() << endl;
    cloud_inflat->width = cloud_inflat->size();
    cloud_inflat->height = 1;
    cloud_normal->width = cloud_normal->size();
    cloud_normal->height = 1;
    // 获取结束时间
    auto end = std::chrono::high_resolution_clock::now();
    // 计算耗时
    std::chrono::duration<double> duration = end - start;
    std::cout << "one frame consume: " << duration.count() << " 秒" << std::endl;

    return cloud_inflat;
    // return cloud_normal;
}

int main(int argc, char **argv)
{
    // 距离射线的距离阈值以及分布在该阈值内的点个数
    DISTANCE_THRESHOLD = 0.08;
    PTS_COUNT_THRESHOLD = 4;
    std::cout << "DISTANCE_THRESHOLD : " << DISTANCE_THRESHOLD << std::endl;
    std::cout << "PTS_COUNT_THRESHOLD : " << PTS_COUNT_THRESHOLD << std::endl;

    std::string work_dir = "/opt/csg/slam/navs/ls/";
    std::cout << "Your work dir is : " << work_dir << std::endl;
    // 这个读取的顺序是对的
    std::vector<string> pcd_file;
    scan_dir_get_filename(work_dir, pcd_file);

    // for (size_t cnts = 0; cnts < 1; cnts++)
    for (size_t cnts = 0; cnts < pcd_file.size() ; cnts++)
    {
        auto inflation_pts = get_inflation_points(pcd_file[cnts]);
        std::string inflation_pts_pcd = pcd_file[cnts].insert(pcd_file[cnts].length() - 4, "_inflation");
        +"_rgb.jpg";
        std::cout << "save inflation_pts_pcd is : " << inflation_pts_pcd << std::endl;
        pcl::io::savePCDFileASCII(inflation_pts_pcd, *inflation_pts);
    }

    return 1;

}