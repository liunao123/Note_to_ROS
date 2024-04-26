#include <iostream>
#include <fstream>
#include <string>

int main() {
    std::string inputFilename = "/home/liunao/map.xml";
    std::string outputFilename = "/home/liunao/map1.xml";
    double subtractValue_x = -21.506353;
    double subtractValue_y = -20.119173;
    double subtractValue_yaw = 0 ; // -3.1415926 * 180.0 /  3.1415926;

    // 打开输入文件
    std::ifstream inputFile(inputFilename);
    if (!inputFile) {
        std::cerr << "Failed to open input file." << std::endl;
        return 1;
    }

    // 打开输出文件
    std::ofstream outputFile(outputFilename);
    if (!outputFile) {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }

    std::string line;
    int cunts = 0;

    while (std::getline(inputFile, line)) 
    {
      cunts++;
      if ( cunts < 8 )
      {
        outputFile << line << std::endl;
        continue;
      }

    std::cout << "cunts : " << cunts << std::endl;
    std::cout << line << std::endl;
      
        size_t pos_ang = line.find("angle=");
        if (pos_ang != std::string::npos) 
        {
                double yawValue = std::stod(line.substr(pos_ang + 7, 6 ));
                yawValue -= subtractValue_yaw;
                line.replace(pos_ang + 7, 5, std::to_string(yawValue) );
        }

        size_t pos = line.find("<pos y=");
        if (pos != std::string::npos) 
        {
                double yValue = std::stod(line.substr(pos + 8, 9 ));
                yValue -= subtractValue_y;
                double xValue = std::stod(line.substr(pos + 22, 9 ));
                xValue -= subtractValue_x;

                line.replace(pos + 8, 9, std::to_string(yValue) );
                line.replace(pos + 22, 9, std::to_string(xValue) );
        }
                outputFile << line << std::endl;
    std::cout << line << std::endl;

    } 

    std::cout << "File processing completed." << std::endl;

    return 0;
}