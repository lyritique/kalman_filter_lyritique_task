# include "data_interaction.hpp"
# include <opencv2/calib3d.hpp>
















namespace pnp {

namespace{




}

//先直接cv的Solver
void pnpSolver::easySolver()
{
    PnPResult pose;
    std::vector<cv::Mat> rvecs{}, tvecs{};//旋转向量，平移向量
    std::vector<double> errs;

    //获得两解rvecs{}, tvecs{}与errs
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


    
    



}
}