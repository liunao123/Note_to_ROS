#include <string>
#include <iostream>

#include <stdlib.h>
#include <unistd.h>
#include <thread>

using namespace std;

int main(int argc, char **argv) 
{

  std::string mv_file ;

  // system("source /opt/csg/slam/navs/devel/setup.bash; roslaunch faster_lio_xn.launch");

  // mv_file = "mv  /home/map/faster_lio_path.txt  /opt/csg/slam/navs/livos/faster_lio_path_xn.txt";
  // system(mv_file.c_str());
  // mv_file = "mv  /opt/csg/slam/navs/livos/result/map_10cm.pcd  /opt/csg/slam/navs/livos/result/faster_lio_xn_.pcd";
  // system(mv_file.c_str());


  printf("-----------------------EXIT------------------\n\n\n");

  // system("source /opt/csg/slam/navs/devel/setup.bash; roslaunch fast_livo_xn.launch");

  // mv_file = "mv  /home/map/fast_livo_body_path.txt  /opt/csg/slam/navs/livos/fast_livo_xn.txt";
  // system(mv_file.c_str());
  // system( " rosservice call /save_map ");
  // sleep(100);

  // mv_file = "mv  /opt/csg/slam/navs/livos/result/map_10cm.pcd  /opt/csg/slam/navs/livos/result/fast_livo_xn.pcd";
  // system(mv_file.c_str());
  // # fdjfhd


  printf("-----------------------EXIT------------------\n\n\n");
  system(" roslaunch /opt/csg/slam/navs/livos/r3live_xn.launch ");

  mv_file = "mv  /home/map/r3live_body_path.txt  /opt/csg/slam/navs/livos/r3live_xn.txt";
  system(mv_file.c_str());
  system( " rosservice call /save_map ");
  sleep(100);
  mv_file = "mv  /opt/csg/slam/navs/livos/result/map_10cm.pcd  /opt/csg/slam/navs/livos/result/r3live_xn.pcd";
  system(mv_file.c_str());
  system( " rosnode kill /laserMapping ");


  return 0;


}