# include <eigen3/Eigen/Eigen>
#include <vector>

namespace kalfilter 
{


struct imu 
{
    double time;
    double q_w;
    double q_x;
    double q_y;
    double q_z;
};

std::vector<imu> readImuTxt(std::string file_address);




}
