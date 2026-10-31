# ifndef __KALMAN__
# define __KALMAN__
# include <eigen3/Eigen/Eigen>
# include <vector>




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



    double assoc_pos_gate {50};
    double chi2_gate {9.49};

};

//相机系球坐标，用于求R观测噪声协方差矩阵，噪声天然的坐标系是 YPD
struct cam_ypd
{
    double azi {};
    double ele {};
    double dist {};
};


struct ObsAssociation {                     
    int    id = -1;                            // 匹配到的 id
    double d2 = std::numeric_limits<double>::infinity();
    bool   valid = false;                      // 是否通过门限
    Eigen::Matrix<double,4,1> nu  {};              // 残差（yaw 已 wrap）
    Eigen::Matrix<double,4,9> H   {};               // 雅可比
    Eigen::Matrix4d           Sinv{};            // S 的逆，供更新复用
};

struct armerState 
{
    Eigen::Matrix<double,4,1> measurement;
    kalfilter::ObsAssociation testResult;
};

struct storedID 
{
    int  id {};      // 0~3
    bool isThisFrame =false;
};

struct KFResult
{
    //px, py, pz, yaw, vx, vy, vz, vyaw, r
    Eigen::Matrix<double, 9, 1> state =Eigen::Matrix<double, 9, 1>::Zero();
    bool valid = false;
};

class KalmanFilter
{
    public:
        explicit KalmanFilter(const KalmanConfig& config = KalmanConfig{});

        void init(armerState& measurement);
        void reset();
        void predict(const double& dt);
        void update(armerState& armor ,const cam_ypd& ypd);
        //这个是每一帧的结束处理id_list的
        void renewArmorList ();
        KFResult getOutput() const;
    private:
        
        
        Eigen::Matrix<double,9,1> X {};  //X=(px,py,pz,yaw,vx,vy,vz,vyaw,r)
        Eigen::Matrix<double,9,9> P {};
        Eigen::Matrix<double,9,9> F {};

        bool initialized = false;

        KalmanConfig config {};

        std::vector<storedID> id_list {};
        int max_id {};


        
};


}//namespace kalfilter
# endif