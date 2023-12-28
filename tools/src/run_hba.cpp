#include <string>
#include <iostream>

#include <stdlib.h>
#include <unistd.h>
#include <thread>

using namespace std;

int main(int argc, char **argv) {

  std::string mv_file ;
  std::string work_dir  = "/home/map/region2-6/";

  for(int i=1; i < 6; i++)
  {
    mv_file = work_dir + " HBA_pose_" + std::to_string(i) + "_.txt";

    system(" source /opt/csg/slam/navs/devel/setup.bash ");
    system(" roslaunch  hba  hba.launch ");

    mv_file = "cp  "  + work_dir + "HBA_pose.txt  " + mv_file;
    std::cout << mv_file << std::endl;
    system(mv_file.c_str());

    sleep(1);
    printf("-----------------------EXIT------------------\n\n\n");
    system(" rosnode kill /hba ");

  }



  printf("-----------------------EXIT------------------\n\n\n");




  return 0;


}