# ifndef __KALMAN__
# define __KALMAN__
# include <eigen3/Eigen/Eigen>




namespace kalfilter {

struct KalmanConfig
{
    //Q
    double p_coord = 100;
    double p_yaw = 400;
    double p_r {};

    //R
    //yac对角噪声
    double ele {0.063};
    double azi {0.063};
    double k_dist {0.05};
    double yaw {0.3};

};

//相机系球坐标，用于求R观测噪声协方差矩阵，噪声天然的坐标系是 YPD
struct cam_ypd
{
    double azi {};
    double ele {};
    double dist {};
};

struct armerState 
{
    Eigen::Matrix<double,4,1> measurement;
    int id ;

  
};

class KalmanFilter
{
    public:
        explicit KalmanFilter(const KalmanConfig& config = KalmanConfig{});

        void init(const double measurement[4]);
        void reset();
    private:
        
        
        Eigen::Matrix<double,9,1> X {};  //X=(px,py,pz,yaw,vx,vy,vz,vyaw,r)
        Eigen::Matrix<double,9,9> P {};
        Eigen::Matrix<double,9,9> F {};

        bool initialized = false;
        void predict(const double& dt,const KalmanConfig& config);
        // wrap=true 时，新息按 [-pi, pi] 环绕（只给 yaw 用）
        void update(armerState armor ,KalmanConfig config ,cam_ypd ypd);
        
};


}
# endif