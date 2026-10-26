# include <eigen3/Eigen/Eigen>

struct KalmanConfig
{

};

struct TargetState {
  // 位置（世界系，m）
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
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

    private:
        struct Kalman1D 
        {
            double p = 0.0;
            double v = 0.0;
            double P00 = 0.0, P01 = 0.0, P10 = 0.0, P11 = 0.0;
            bool initialized = false;

            void init(double measurement, double position_var, double velocity_var);
            void predict(double dt, double q);
            // wrap=true 时，新息按 [-pi, pi] 环绕（只给 yaw 用）
            void update(double measurement, double r, bool wrap);
        };
};