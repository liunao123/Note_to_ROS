#include <string>
#include <vector>
#include <queue>
#include <iostream>

#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <csignal>
#include <string.h>


using namespace std;

bool    g_bExit = false;

void PressEnterToExit( )
{
    // system("source /opt/csg/LX100/navs/devel/setup.bash; roslaunch hdl_localization load_global_map.launch &");
    int c;
    while ( (c = getchar()) != 'c' && c != EOF );
    fprintf( stderr, "\nPress 'c' then Press enter to exit.\n");
    while( getchar() != 'c');
    g_bExit = true;
}

void handle_signal(int signum) {
    printf("\n\n\n\nexit(1)\n\n\n\n\n"); 

    exit(1);
}



int main(int argc, char **argv) 
{  
  // queue<int> tv;
  // tv.push(6);
  // std::cout << "tv.size() :" << tv.size() << std::endl;
  // tv.pop();
  // std::cout << "tv.size() :" << tv.size() << std::endl;
  // tv.pop();
  // std::cout << "tv.size() :" << tv.size() << std::endl;
  // tv.pop();
  // std::cout << "tv.size() :" << tv.size() << std::endl;
  // return 1;

  int num = 0;

  // std::thread measurement_process{PressEnterToExit};
  // measurement_process.detach();

  std::signal(SIGINT, handle_signal);
  sleep( 3 );

  while ( num++ < 5 )
  {
    std::string mv_file = "mv /home/map/hdl_pose.txt  /home/map/hdl_pose_changzhou_odom_" + std::to_string( num )  + ".txt ";

    // system("source /opt/csg/LX100/navs/devel/setup.bash; roslaunch hdl_localization hdl_localization.launch");
    system("source /home/liunao/csg/SH100/slam/navs/devel/setup.bash; roslaunch hdl_localization hdl_localization.launch");

    std::cout << mv_file << std::endl;

    system(mv_file.c_str());

    // mv_file = "mv /home/map/fast_livo_body_path.txt  /home/map/test/changzhou_fast_livo_odom_" + std::to_string( num )  + ".txt ";
    // system(mv_file.c_str());

    printf("sleep 3 seconds. \n\n\n\n\n");
    sleep( 3 );
  }

  printf("-----------------------EXIT------------------\n\n\n");

  return 0;
}
