#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>

#include <sys/types.h>
#include <dirent.h>
 
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/time.h>

typedef pcl::PointXYZRGB PointTypeRGB;
// typedef pcl::PointXYZI PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

using namespace std;
using namespace Eigen;

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
    std::string file_type = std::string(entry->d_name);

		// std::cout << "file_type  is : " << file_type << std::endl;
    if (file_type.length() <= 7 )
    {
      continue;
    }
    
    if ( file_type.substr(file_type.length() - 3 ) == "ply" )
    {
		  filenames.push_back( path + file_type );
    }
    
		free(entry);
	}
	std::cout << "filenames size is : " << filenames.size() << std::endl;
	free(entry_list);
}


int main(int argc, char **argv)
{
  std::string path = "/home/liunao/Kalibr/v1_20241118/pre1/";
  double voxel_size =  0.05;
  if (argc == 3)
  {
    path = argv[1];
    voxel_size = std::stof(argv[2]);
  }

  std::cout << "path1: " << path << std::endl;
  std::cout << "argc: " << argc << std::endl;

  vector<string> filenames;
  scan_dir_get_filename( path, filenames );

  pcl::PLYReader reader;
  
  PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr filtered_cloud(new PointCloudXYZRGB);

  pcl::VoxelGrid<PointTypeRGB> downSizeFilterTempMap;
  downSizeFilterTempMap.setLeafSize(voxel_size , voxel_size , voxel_size  );

  for (size_t i = 0; i < filenames.size(); i++)
  {
    std::cout << "file is : " << filenames[i] << std::endl;
    if (reader.read(filenames[i], *cloud) == -1)
    {
      PCL_ERROR("Couldn't read the PLY file \n");
    }

    // if (pcl::io::loadPCDFile(filenames[i], *cloud) == -1)
    // {
    //   PCL_ERROR("Couldn't read pcd file . \n");
    //   return (-1);
    // }

    std::cout << "Saved voxel before: " << cloud->size() << " data points to test_pcd.pcd." << std::endl;
    downSizeFilterTempMap.setInputCloud(cloud);
    downSizeFilterTempMap.filter(*filtered_cloud);
    std::cout << "Saved voxel after: " << filtered_cloud->size() << " data points to test_pcd.pcd." << std::endl;

    // pcl::io::savePLYFile("/home/output_cloud.ply", *filtered_cloud);

    size_t last_position = filenames[i].find_last_of('/') ;
    size_t last_position_2 = filenames[i].find_last_of('.') ;
    // std::string plyfile =  filenames[i].substr(last_position + 1,  filenames[i].length() - last_position ) ;
    std::string plyfile =  "/home/liunao/RandLA-Net/data/semantic3d/downsample_ply/" + filenames[i].substr(last_position + 1,  last_position_2 - last_position )  + "ply" ;
    std::cout << "plyfile 112: " << plyfile << std::endl;
    // RandLA-Net\data\semantic3d
    pcl::io::savePLYFile( filenames[i] , *filtered_cloud );
    // pcl::io::savePLYFile( plyfile , *filtered_cloud );


  }
  return (0);

}
