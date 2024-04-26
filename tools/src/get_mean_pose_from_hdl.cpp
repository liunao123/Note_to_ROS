#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>

#include <unordered_set>

using namespace std;
using namespace Eigen;
typedef Matrix< float , Eigen::Dynamic , 8 > Vector8f;


template <typename typeT>
class Solution {
public:
    vector<typeT> intersection(vector<typeT>& nums1, vector<typeT>& nums2) {
        unordered_set<typeT> set1, set2;
        for (auto& num : nums1) {
            set1.insert(num);
        }
        for (auto& num : nums2) {
            set2.insert(num);
        }
        cout << " size is 35   " << set1.size() << "  "  << set2.size() << endl ;

        return getIntersection(set1, set2);
    }

    vector<typeT> getIntersection(unordered_set<typeT>& set1, unordered_set<typeT>& set2) {
        if (set1.size() > set2.size()) {
            return getIntersection(set2, set1);
        }

        vector<typeT> ist;

        for (auto& num : set1) {
            if ( set2.count(num) ) {
                ist.push_back(num);

            }
        }
        return ist;
    }
};



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

int main()
{
  string input_path = "/home/map/test/xn/t/";          // 输入文件名
  string output_file = "/home/liunao/mean.txt"; // 输出文件名

  ofstream fout(output_file);

  vector<string> hdl_file;
  scan_dir_get_filename(input_path, hdl_file);

  std::vector< Vector8f > pose_mat;
  pose_mat.resize( hdl_file.size() );

  std::vector< std::vector< double > > time_vector;
  time_vector.resize( hdl_file.size() );

  string line;
  for (int i = 0; i < hdl_file.size() ; i++)
  {
    ifstream fin(hdl_file[i]);
    int row = 0;

    pose_mat[i].resize(100000,8);
    while (getline(fin, line))
    {
      string out;
      istringstream ss(line);
      string word;
      int j=0;
      while (ss >> word)
      {
        pose_mat[i]( row ,j++) = std::stof(word);
        if(j==1)
        {
          time_vector[i].push_back( std::stof(word) );
        }
      }
      row++;
    }
    fin.close();
  }

  fout.close();

  Solution< double > st;
  vector< double > result;
  result = time_vector[0];

  for (int i = 1; i < time_vector.size() ; i++)
  {
    result = st.intersection(result , time_vector[i]);
  }

  cout << "same timestamps size is: " << result.size()  << endl ;

  for (int i = 0; i < result.size() ; i++)
  {
    cout << result[i]  << endl ;
  }

  return 0;
}