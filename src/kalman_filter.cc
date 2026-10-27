# include "kalman_filter_task.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
# include <eigen3/Eigen/Eigen>


/*
EKF追踪X=(px,py,pz,yaw,vx,vy,vz,vyaw,r)
predict X_=f(X),P_=F*P*F^T+Q ,F=f(X)一阶导
新息    h(X_,id)从八维变为四维装甲板数据，y=z-h(X_)，S=H*P_*H^T+R，H=h(X_)一阶导
update  增益K=P*H^T*S^-1，  △X=K*y，  X^+=X_+△X
*/

namespace kalfilter 
{
    namespace 
    {

        constexpr double kPI = 3.14159265358979323846;

        void wrap ()
        {

        }

        //h(X_),是九维列向量转换为四维列向量，从描述中心到描述装甲板
        Eigen::Matrix<double,4,1> observationModel(const Eigen::Matrix<double,4,1>& X_,int id )
        {
            Eigen::Matrix<double,4,1> y;
            y << X_(0)+X_(8)*std::sin(X_(3)+id*kPI*0.5),
             X_(1),
             X_(2)+X_(8)*std::cos(X_(3)+id*kPI*0.5),
             X_(3)+id*kPI*0.5;

            return y;
        }

        //计算新息雅克布矩阵H
        Eigen::Matrix<double,4,9> computeH(const Eigen::Matrix<double,9,1>& X_, int id)
        {
            const double c = id * kPI * 0.5;
            const double theta = X_(3) + c;
            const double s = std::sin(theta);
            const double co = std::cos(theta);
            const double r = X_(8);

            Eigen::Matrix<double,4,9> J {};
        

            J(0,0) = 1.0;
            J(0,3) =  r * co;
            J(0,8) =  s;

            J(1,1) = 1.0;

            J(2,2) = 1.0;
            J(2,3) = -r * s;
            J(2,8) =  co;

            J(3,3) = 1.0;

            return J;
        }

        Eigen::Matrix4d computeR(const KalmanConfig& config,const cam_ypd ypd)
        {
            //预计算
            const double s_e {std::sin(ypd.ele)};
            const double c_e {std::cos(ypd.ele)};
            const double s_a {std::sin(ypd.azi)};
            const double c_a {std::cos(ypd.azi)};

            /*构造相机系下ypd->直角系的雅可比J
            x = dist · cos(ele) · sin(azi)       右
            y = dist · sin(ele)                  下
            z = dist · cos(ele) · cos(azi)       前（光轴）
            yaw无变换
            */
            Eigen::Matrix4d J {};
            J.col(0) =  Eigen::Vector3d(ypd.dist*c_e*c_a,  -ypd.dist*s_e*s_a,  c_e*s_a);      
            J.col(1) =  Eigen::Vector3d(0.0,               ypd.dist*c_e,       s_e);      
            J.col(2) =  Eigen::Vector3d(-ypd.dist*c_e*s_a, -ypd.dist*s_e*c_a,  c_e*c_a);
            J(3,3) = 1.0;

            //深度噪声
            const double sigma_dist = config.k_dist * ypd.dist;

            //构造R_ypd，理论4*4，但对角，就写成向量吧
            Eigen::Matrix<double,4,1> R_ypd(config.azi*config.azi,config.ele*config.ele,
                                    sigma_dist*sigma_dist,config.yaw*config.yaw);

            return  Eigen::Matrix4d   { J * R_ypd.asDiagonal() * J.transpose()};
            



        }

    }
    
    void KalmanFilter::init(const double *measurement)
    {
        F.setIdentity();
    }


    void KalmanFilter::reset()
    {
        initialized = false;
        X = Eigen::Matrix<double,9,1> {};
        P = Eigen::Matrix<double,9,9> {};
    }
   

    void KalmanFilter::predict(const double& dt,const KalmanConfig& config)
    {
        //f(X)部分，由于后面不再用到t-1的X^+，直接覆盖
        /*我的旧写法
        X(0) += X(4)*dt;
        X(1) += X(5)*dt;
        X(2) += X(6)*dt;
        X(3) += X(7)*dt;
        */
        X.head<4>() += X.segment<4>(4)*dt;

        //计算雅克布矩阵
        //让它特定位置的4*4块变成单位乘dt，就是根据dt更新了
        F.template block<4,4>(0,4) = dt * Eigen::Matrix4d::Identity();

        //分段白噪声模型制造Q
        //模型中目标的加速度是一个分段常数，在每一段内以为白噪声形式随机变化
        Eigen::Matrix<double, 9, 9> Q {};
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        auto fill_block = [&](int i, int j, double p) 
        {
            Q(i,i) = p * dt4 / 4.0;
            Q(i,j) = p * dt3 / 2.0;
            Q(j,i) = p * dt3 / 2.0;
            Q(j,j) = p * dt2;
        };//lambda看着真舒服，[&]后不用多写输入

        fill_block(0,4, config.p_coord);
        fill_block(1,5, config.p_coord);
        fill_block(2,6, config.p_coord);
        fill_block(3,7, config.p_yaw);
        Q(8,8) = config.p_r * dt; 

        P = F * P * F.transpose() + Q;
        
        //正定化，就是沿对角线对称
        P = 0.5 * (P + P.transpose());

    }


    void KalmanFilter::update (armerState armor ,KalmanConfig config ,cam_ypd ypd)
    {
        //h(X_)部分写在匿名命名空间
        //计算残差
        Eigen::Matrix<double,4,1> residual {armor.measurement-observationModel(X,armor.id)};

        //计算新息雅克布矩阵H写在匿名命名空间

        //观测噪声协方差矩阵 R 写在匿名命名空间
        //基于上科大开源技术

        Eigen::Matrix4d H {computeH(X, armor.id)};
        Eigen::Matrix4d R {computeR(config, ypd)};

        //计算新息协方差矩阵S
        Eigen::Matrix4d S {H * P * H.transpose() + R};
        
        //kalman增益K
        Eigen::MatrixXd K {P * H.transpose()*S.inverse()};

        //更新X，P
        X += K * residual;
        Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(9, 9) - K * H;//就是I-KH
        P = IKH * P * IKH.transpose() + K * R * K.transpose(); 
    }
}




