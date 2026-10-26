# include "imu_alignment.hpp"
# include <eigen3/Eigen/src/Core/Matrix.h>
# include <fstream>
# include <eigen3/Eigen/Dense>
# include <vector>
# include <sstream>
# include <iostream>





//读取imu数据，直接读出一个imu数据的组
std::vector<kalfilter::imu> readImuTxt(std::string file_address)
{
    std::ifstream fin(file_address);
    //判断是否打开成功
    if (!fin.is_open())
    {
        throw std::runtime_error("打不开 IMU 文件: " + file_address);
        //std::cerr << "无法打开文件\n";
    }

    //逐行读取txt内容，直到getline读取错误
    std::vector<kalfilter::imu> imu_data;

    std::string line;
    int lineNo {};
    while (std::getline(fin,line))
    {
        ++lineNo;
        if (line.empty()) continue;

        std::istringstream iss(line);
        kalfilter::imu r;
        if (iss >> r.time >> r.q_w >> r.q_x >> r.q_y >> r.q_z) {
            imu_data.push_back(r);
        } else {
            std::cerr << "第 " << lineNo << " 行格式不符，跳过\n";
        }

    }

    
    return imu_data;

}







