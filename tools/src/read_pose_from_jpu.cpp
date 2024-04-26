#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <chrono>

using namespace std;

int main()
{
  string search_term = "robot_real_pos() x = ";          // 要查找的文本
  string input_file = "/home/liunao/jpu_2023-04-26.log"; // 输入文件名
  string output_file = "/home/liunao/robot_real_pos.txt";        // 输出文件名

  ifstream fin(input_file);
  ofstream fout(output_file);

  string line;
  while (getline(fin, line))
  {
    if (line.find(search_term) != string::npos)
    {
      size_t pos_x = line.find("x = ");
      size_t pos_y = line.find("y = ");
      // size_t pos_th = str.find("theta = ");
      if (pos_x != string::npos && pos_x != string::npos)
      {
        string time = line.substr(1, 19);
        string x = line.substr(pos_x + 4, 8);
        string y = line.substr(pos_y + 4, 8);

        // string str = "2023-04-26 12:34:56";

        // 解析时间字符串
        std::tm tm = {};
        std::istringstream ss(time);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        // 转换为Unix时间戳
        auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));
        auto ts = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch()).count();

        cout << "time " << ts << "  x " << x << " " << y << endl;
        fout << ts << " " << x << " " << y << " 0 0 0 0 1" << endl;
      }
    }
  }

  fin.close();
  fout.close();

  return 0;
}