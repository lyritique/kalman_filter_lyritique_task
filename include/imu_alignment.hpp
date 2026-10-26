# ifndef _IMU_
# define _IMU_

# include <eigen3/Eigen/Eigen>
# include <vector>

namespace kalfilter 
{


struct imu 
{
    double time;
    Eigen::Quaterniond q;
};

std::vector<imu> readImuTxt(std::string file_address);




}


# endif
