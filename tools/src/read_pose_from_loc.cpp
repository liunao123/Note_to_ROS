#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <chrono>

using namespace std;

int main()
{
  string search_term = "fixed curPose";          // 要查找的文本
  string input_file = "/opt/csg/slam/navs/zz_roslog_0811/loc_2024-08-11.log"; // 输入文件名
  string output_file = "/home/robot_real_pos_loc.txt";        // 输出文件名

  ifstream fin(input_file);
  ofstream fout(output_file);

  string line;
  while (getline(fin, line))
  {
    if (line.find(search_term) != string::npos)
    {
      size_t pos_x = line.find("fixed curPose x = ");
      size_t pos_y = line.find(",y = ");
      size_t pos_yaw = line.find(",yaw = ");
        cout << "27: " << pos_x << " x " << pos_y  << " " << pos_yaw << endl;

      // size_t pos_th = str.find("theta = ");
      if (pos_x != string::npos && pos_x != string::npos)
      {
        string time = line.substr(1, 19);
        string x = line.substr(pos_x + 18, pos_y - pos_x - 18);
        string y = line.substr(pos_y + 5, pos_yaw - pos_y - 5);
        string yaw = line.substr(pos_yaw + 7, 8 );

        // string str = "2023-04-26 12:34:56";

        // 解析时间字符串
        std::tm tm = {};
        std::istringstream ss(time);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        // 转换为Unix时间戳
        auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));
        auto ts = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch()).count();

        cout << "time " << ts << "  x " << x << " " << y << endl;
        fout << ts << " " << x << " " << y << " " << yaw << " 0 0 0 1" << endl;
      }
    }
  }

  fin.close();
  fout.close();

  return 0;
}