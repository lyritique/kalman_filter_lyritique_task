# include <eigen3/Eigen/Eigen>




namespace kalfilter {

struct KalmanConfig
{
    //Q
    double p_coord = 100;
    double p_yaw = 400;
    double p_r {};
};

struct aromerState {
  // 位置（世界系，m）
  double px {};
  double py {};
  double pz {};
  double yaw = 0.0;  // 朝向（rad）

  // 速度（世界系，m/s 与 rad/s）
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double vyaw = 0.0;

  bool valid = false;
  std::string reason;
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
        void update(double measurement, double r, bool wrap);
        
};


}