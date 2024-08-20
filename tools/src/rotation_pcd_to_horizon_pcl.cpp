#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>

int main(int argc, char **argv)
{
  std::string pcd_file;
  if (argc > 1)
  {
    pcd_file = argv[1];
  }
  else
  {
    PCL_ERROR("you need given a pcd file .\n");
    PCL_ERROR("USAGE: rotation_pcd_to_horizon_pcl pcd_file. \n");
    PCL_ERROR("ex: rotation_pcd_to_horizon_pcl /home/1.pcd \n");
    return -1;
  }
  std::cout << "Point cloud file is \"" << pcd_file << "\"\n";
  pcl::PointCloud<pcl::PointXYZ> cloud;

  pcl::io::loadPCDFile(pcd_file, cloud);
  printf("size of cloud map: %ld . \n", cloud.points.size());

  // 创建一个模型参数对象，用于记录结果
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // inliers表示误差能容忍的点 记录的是点云的序号
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // 创建一个分割器
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
  seg.setOptimizeCoefficients(true);
  // Mandatory-设置目标几何形状
  seg.setModelType(pcl::SACMODEL_PLANE);
  // 分割方法：随机采样法
  seg.setMethodType(pcl::SAC_RANSAC);
  // 设置误差容忍范围，也就是我说过的阈值
  seg.setDistanceThreshold(0.1);
  // 输入点云
  seg.setInputCloud(cloud.makeShared());
  // 分割点云
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset. EXIT . ");
    return (-1);
  }

  std::cout << "get planar model size: " << inliers->indices.size() << std::endl;

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

  // float select_x = 14.43;
  // float select_y = 33.3;

  // float range = 0.50;
  // pcl::PointCloud<pcl::PointXYZ> cloud_in_1m;
  // static pcl::CropBox<pcl::PointXYZ> cropBoxFilter_temp(true);
  // cropBoxFilter_temp.setInputCloud(cloud.makeShared());
  // cropBoxFilter_temp.setMin(Eigen::Vector4f(select_x - range, select_y - range, -1.0, 1.0f));
  // cropBoxFilter_temp.setMax(Eigen::Vector4f(select_x + range, select_y + range, 1.3, 1.0f));
  // cropBoxFilter_temp.setNegative(false);
  // cropBoxFilter_temp.filter(cloud_in_1m);
  // std::cout << "cloud_in_1--m size: " << cloud_in_1m.points.size() << std::endl;

  // auto plan_file_temp = pcd_file;
  // plan_file_temp.insert(plan_file_temp.size() - 4, "_use_plan");
  // pcl::io::savePCDFileASCII(plan_file_temp, cloud_in_1m );

  // if (cloud_in_1m.points.size() > 20)
  // {
  //   // 输入点云
  //   seg.setInputCloud(cloud_in_1m.makeShared());
  //   // 分割点云
  //   seg.segment(*inliers, *coefficients);

  //   if (inliers->indices.size() == 0)
  //   {
  //     PCL_ERROR("Could not estimate a planar model for the given dataset. EXIT . ");
  //     return (-1);
  //   }

  //   std::cout << "get planar model size: " << inliers->indices.size() << std::endl;

  //   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  //             << coefficients->values[1] << " "
  //             << coefficients->values[2] << " "
  //             << coefficients->values[3] << std::endl;
  // }
  // 参考: https://blog.csdn.net/weixin_38636815/article/details/109543753

  // 首先求解出旋转轴和旋转向量
  // a,b,c为求解出的拟合平面的法向量，是进行归一化处理之后的向量。
  Eigen::Vector3d plane_norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

  // xz_norm是参考向量，也就是XOY坐标平面的法向量
  Eigen::Vector3d xz_norm(0.0, 0.0, 1.0);

  // 求解两个向量的点乘
  double v1v2 = plane_norm.dot(xz_norm);

  // 计算平面法向量和参考向量的模长，因为两个向量都是归一化之后的，所以这里的结果都是1.
  double v1_norm = plane_norm.norm();
  double v2_norm = xz_norm.norm();
  // 计算两个向量的夹角
  double theta = std::acos(v1v2 / (v1_norm * v2_norm));
  std::cout << "theta <rad> is  : " << theta << std::endl;
  std::cout << "theta <deg> is  : " << theta * 180.0 / 3.14159 << std::endl;

  // 根据向量的叉乘求解同时垂直于两个向量的法向量。
  Eigen::Vector3d axis_v1v2 = xz_norm.cross(plane_norm);

  // 对旋转向量进行归一化处理
  axis_v1v2 = axis_v1v2 / axis_v1v2.norm();

  // 计算旋转矩阵
  Eigen::AngleAxisd ro_vector(-theta, Eigen::Vector3d(axis_v1v2.x(), axis_v1v2.y(), axis_v1v2.z()));
  Eigen::Matrix3d ro_matrix = ro_vector.toRotationMatrix();
  std::cout << "ro_matrix eigen: " << std::endl << ro_matrix << std::endl;

  pcl::PointCloud<pcl::PointXYZ> flat_cloud;
  flat_cloud = cloud;
  flat_cloud.points.clear();

  printf("processing ");

  for (int i = 0; i < cloud.points.size(); i++)
  {
    Eigen::Vector3d newP(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    Eigen::Vector3d new_point = ro_matrix * newP;
    pcl::PointXYZ pt;
    pt.x = new_point.x();
    pt.y = new_point.y();
    pt.z = new_point.z();
    flat_cloud.points.push_back(pt);
    int process = int(100 * double(i) / cloud.points.size());
    if (i % int(cloud.points.size() / 5) == 0)
      std::cout << process << "% ... " << std::endl;
  }
  std::cout << "100% ... " << std::endl;


  // 计算提取出的平面点的 平均高度，后面可以统一减去这个值
  double mean_z = 0;
  for (int i = 0; i < inliers->indices.size(); i++)
  {
    mean_z += flat_cloud.points[inliers->indices[i]].z;
  }
  mean_z = mean_z / inliers->indices.size();
  std::cout << "--------------------------------- " << std::endl;
  std::cout << "mean_z: " << mean_z << std::endl;
  std::cout << "--------------------------------- " << std::endl;

  for (int i = 0; i < flat_cloud.points.size(); i++)
  {
    flat_cloud.points[i].z -= mean_z;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extract the inliers
  // 把归一化后的平面点再提取一次
  extract.setInputCloud ( flat_cloud.makeShared() );
  extract.setIndices (inliers);
  extract.setNegative (false);   //如果设为true,可以提取指定index之外的点云
  extract.filter (*cloud_p);
  auto plan_file = pcd_file;
  plan_file.insert(plan_file.size() - 4, "_plan");
  pcl::io::savePCDFileASCII(plan_file, *cloud_p);

  std::cout << "flat_cloud size: " << flat_cloud.points.size() << std::endl;

  pcd_file.insert(pcd_file.size() - 4, "_horizontal");
  pcl::io::savePCDFileASCII(pcd_file, flat_cloud);
  std::cout << "save result pcd :pcd_file_horizontal.pcd  >>  " << pcd_file << std::endl;

  cloud.points.clear();
  cloud =  flat_cloud;
  std::cout << "--------------- rotation cloud size: " << cloud.points.size() << std::endl;

}
