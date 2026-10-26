# ifndef __IMU__
# define __IMU__

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


kalfilter::imu interpolateQuat(const std::vector<kalfilter::imu>& imu_data,double camTime);


}


# endif
