# include "kalman_filter_task.hpp"
#include <Eigen/src/Core/Matrix.h>
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
        void wrap ()
        {

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
        };//lambda看着真舒服，[&]不用多写

        fill_block(0,4, config.p_coord);
        fill_block(1,5, config.p_coord);
        fill_block(2,6, config.p_coord);
        fill_block(3,7, config.p_yaw);
        Q(8,8) = config.p_r * dt; 

        P = F * P * F.transpose() + Q;
        
        //正定化，就是沿对角线对称
        P = 0.5 * (P + P.transpose());

    }
}




