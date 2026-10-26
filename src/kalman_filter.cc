# include "kalman_filter_task.hpp"
# include <eigen3/Eigen/Eigen>

int main ()
{

}



namespace kalman_filter 
{
    namespace 
    {


    }
    
    
   
}
void KalmanFilter::Kalman1D::init(double measurement, double position_var,
                                       double velocity_var) {
  p = measurement;
  v = 0.0;  // 第一帧没有速度信息
  P00 = position_var;
  P01 = 0.0;
  P10 = 0.0;
  P11 = velocity_var;
  initialized = true;
}