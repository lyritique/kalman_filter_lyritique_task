# include "data_interaction.hpp"
# include <opencv2/calib3d.hpp>
# include <opencv2/core.hpp>
# include <iostream>
















namespace pnp {

namespace{




}








pnpSolver::pnpSolver()
{
    const pnpConfig::camParams cam;

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        cam.fx, 0.0,    cam.cx,
        0.0,    cam.fy, cam.cy,
        0.0,    0.0,    1.0);

    std::vector<double> d(cam.dist.data(), cam.dist.data() + cam.dist.size());
    distortion_coeffs_ = cv::Mat(d, true).clone();
}

void pnpSolver::input(
    const std::array<cv::Point2f, 4>& image_corners,
    bool big)
{
    corners.assign(image_corners.begin(), image_corners.end());
    armorPoint = big ? config.big_armorPoint
                     : config.small_armorPoint;

    output.setZero();
    output_valid_ = false;
    proj_err = std::numeric_limits<double>::infinity();
}

//先直接cv的Solver
void pnpSolver::easySolver()
{
    PnPResult pose;
    std::vector<cv::Mat> rvecs{}, tvecs{};//旋转向量，平移向量
    std::vector<double> errs;

    //获得rvecs{}, tvecs{}与errs
    const int nsol = cv::solvePnPGeneric(armorPoint, corners, 
        camera_matrix_,distortion_coeffs_ , rvecs, tvecs,
        false, cv::SOLVEPNP_IPPE,cv::noArray());

    if (nsol <= 0 || rvecs.empty()) 
    {
        //
        return;
    }

    int best = -1;
    double best_err = 1e18;
    double best_cost {10086};
    //cv::Mat best_R {};
    double best_yaw {};
    cv::Mat best_tvec {};
    

    for (int i = 0; i < nsol; ++i)
    {
        //判断是否板跑到相机后面
        double tz = tvecs[i].at<double>(2);
        if (tz <= 0.0) continue;

        cv::Mat R;
        cv::Rodrigues(rvecs[i], R);

        //版应朝向相机
        //if (R.col(0).dot(tvecs[i]) <=0)continue;

        std::vector<cv::Point2d> reproj{};
        //重投影，结果reproj
        cv::projectPoints(armorPoint, rvecs[i], tvecs[i],
                          camera_matrix_, distortion_coeffs_, reproj);

        //算最小二乘式的err
        double squared_error {};
        for (int i = 0; i < 4; ++i) 
        {
            const cv::Point2d diff = reproj[i] - corners[i];
            squared_error += diff.dot(diff);
        }
        const double rms = std::sqrt(squared_error / 4.0);

        const double nx  = R.at<double>(0, 0);
        const double nz  = R.at<double>(2, 0);
        const double yaw = std::remainder(std::atan2(nx, nz),2.0 * M_PI);
        const double dyaw {yaw-previous_yaw};
        double cost {rms + config.kyaw*dyaw*dyaw };

        if (!std::isfinite(rms) || rms > config.max_reproj_err)continue;

        if (cost < best_cost) 
        { 
            best_err = rms; best = i; best_tvec = tvecs[i].clone();
            best_cost = cost; best_yaw = yaw;
        }




        std::cout << "sol " << i
          << " rms=" << rms
          << " n·t=" << R.col(0).dot(tvecs[i])
          << " yaw=" << yaw * 57.3 << '\n';
    }

    if (best_err > config.max_reproj_err) return;
    if (best < 0) return;

    cv::solvePnPRefineLM(
    armorPoint,
    corners,
    camera_matrix_,
    distortion_coeffs_,
    rvecs[best],
    tvecs[best]);

    //再算重投影误差
    std::vector<cv::Point2d> reproj{};
    cv::projectPoints(armorPoint, rvecs[best], tvecs[best],
                          camera_matrix_, distortion_coeffs_, reproj);
    double squared_error {};
    for (int i = 0; i < 4; ++i) 
    {
        const cv::Point2d diff = reproj[i] - corners[i];
        squared_error += diff.dot(diff);
    }
    const double rms = std::sqrt(squared_error / 4.0);

    cv::Mat improved_R;
    cv::Rodrigues(rvecs[best], improved_R);
    const double nx  = improved_R.at<double>(0, 0);
    const double nz  = improved_R.at<double>(2, 0);

    double improved_yaw {std::remainder(std::atan2(nx, nz),2.0 * M_PI)};
    const double dyaw {improved_yaw-previous_yaw};
    double cost {rms + config.kyaw*dyaw*dyaw };

    bool accept_improve = true;
    if (cost >= best_cost) accept_improve = false ;
    

    



    // solvePnP 的平移单位与对象点一致，当前对象点是毫米
    const double tx = (accept_improve ? tvecs[best]:best_tvec).at<double>(0) / 1000.0;
    const double ty = (accept_improve ? tvecs[best]:best_tvec).at<double>(1) / 1000.0;
    const double tz = (accept_improve ? tvecs[best]:best_tvec).at<double>(2) / 1000.0;

    

    output << tx, ty, tz, (accept_improve?improved_yaw:best_yaw);
    proj_err = (accept_improve?rms:best_err) ;
    output_valid_ = true;


    
    



}

void pnpSolver::renewPrevious(double& best_yaw)
{
    previous_yaw = best_yaw;
}


PnPResult pnpSolver::getOutput()
{
    PnPResult result;
    result.valid = output_valid_;
    result.measurement = output;
    result.err = proj_err ;
    return result;
}
}