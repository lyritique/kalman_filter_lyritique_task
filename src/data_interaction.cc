# include "data_interaction.hpp"
# include <opencv2/calib3d.hpp>
# include <opencv2/core.hpp>
















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
        false, cv::SOLVEPNP_IPPE,cv::noArray(), errs);

    if (nsol <= 0 || rvecs.empty()) 
    {
        //
        return;
    }

    int best = -1;
    double best_err = 1e18;

    for (int i = 0; i < nsol; ++i)
    {
        std::vector<cv::Point2d> reproj{};
        //重投影，结果reproj
        cv::projectPoints(armorPoint, rvecs[i], tvecs[i],
                          camera_matrix_, distortion_coeffs_, reproj);

        double err = 0.0;
        for (size_t k = 0; k < reproj.size(); ++k)
            err += cv::norm(reproj[k] - corners[k]);
        err /= static_cast<double>(reproj.size());

        if (std::isfinite(err) && err < best_err) { best_err = err; best = i; }
    }

    if (best_err > config.max_reproj_err) return;
    if (best < 0) return;

    // solvePnP 的平移单位与对象点一致，当前对象点是毫米
    const double tx = tvecs[best].at<double>(0) / 1000.0;
    const double ty = tvecs[best].at<double>(1) / 1000.0;
    const double tz = tvecs[best].at<double>(2) / 1000.0;

    // 从旋转矩阵取装甲板法向
    cv::Mat rotation;
    cv::Rodrigues(rvecs[best], rotation);

    const double nx = rotation.at<double>(0, 0);
    const double nz = rotation.at<double>(2, 0);
    const double yaw = std::atan2(nx, nz);

    output << tx, ty, tz, yaw;
    output_valid_ = true;


    
    



}


PnPResult pnpSolver::getOutput()
{
    PnPResult result;
    result.valid = output_valid_;
    result.measurement = output;
    return result;
}
}