#include <string>
#include <iostream>

#include <stdlib.h>
#include <unistd.h>
#include <thread>

using namespace std;

bool    g_bExit = false;

void PressEnterToExit( )
{
    int c;
    while ( (c = getchar()) != 'c' && c != EOF );
    fprintf( stderr, "\nPress 'c' then Press enter to exit.\n");
    while( getchar() != 'c');
    g_bExit = true;
}


int main(int argc, char **argv) 
{
  int num = 1;

  std::thread measurement_process{PressEnterToExit};
  measurement_process.detach();

  std::string mv_file ;

  while ( !g_bExit )
  {
    std::cout << ++num << "th" << std::endl;

    std::string mv_file = "mv /home/map/r3live_body_path.txt  /home/map/test/r3live_body_path_" + std::to_string( num )  + ".txt ";

    if (num < 10)
    { 
      // mv_file = "mv  /home/map/faster_lio_path.txt  /home/map/test/faster_lio_" + std::to_string( num )  + ".txt ";
      // system("source /opt/csg/slam/navs/devel/setup.bash; roslaunch faster_lio mapping_horizon.launch");
      // ! r2live  r3live  fast_livo  faster_lio
      system("source /home/liunao/csg/SH100/slam/navs/devel/setup.bash; roslaunch fast_livo v1_livox_hk_sync.launch");
      // system("source /opt/csg/slam/navs/devel/setup.bash; roslaunch fast_livo mapping_v1_xn.launch");
      // system("source /opt/csg/slam/navs/5.xRobot_SLAM_Dog/devel/setup.bash ; roslaunch fast_livo mapping_v1_xn.launch");
      // system("source /home/liunao/r3/devel/setup.bash; roslaunch r3live r3live_bag_v1.launch");
    }

    // if (num >  10 && num <  20)
    // { 
      mv_file = "mv /home/map/fast_livo_body_path.txt  /home/map/test/fast_livo_body_path_" + std::to_string( num )  + ".txt ";
    //   system("source /opt/csg/slam/navs/devel/setup.bash; roslaunch fast_livo mapping_v1_2nd.launch");
    // }


    if (num >  20 )
    {
      g_bExit = true;
    }

    std::cout << mv_file << std::endl;

    system(mv_file.c_str());

    // std::string save_param =" rosparam dump /home/map/test/xn_" + std::to_string( num )  + ".yaml ";
    // std::cout << save_param << std::endl;

    // system( save_param.c_str() );
 
    printf("sleep 100 seconds. \n\n\n\n\n");
    sleep(5);
  }

    printf("-----------------------EXIT------------------\n\n\n");

  return 0;


  // give a c++ demp for vector
  
}