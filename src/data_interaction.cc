# include "data_interaction.hpp"
# include <opencv2/calib3d.hpp>
# include <opencv2/core.hpp>
# include <iostream>
















namespace pnp {

namespace{

constexpr double kPi {3.14159265358979323846};

double wrapPi(double a)//我怎么觉得好像写过
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}



}

std::array<cv::Point2f, 4> pnpSolver::canonicalize(
    const std::array<cv::Point2f, 4>& raw, bool reverse) const
{
    cv::Point2f centre {0.0f, 0.0f};
    for (const auto& p : raw) centre += p;
    centre *= 0.25f;

    std::array<int, 4> order {0, 1, 2, 3};
    std::sort(order.begin(), order.end(), [&](int a, int b)
    {
        const double aa = std::atan2(raw[a].y - centre.y, raw[a].x - centre.x);
        const double bb = std::atan2(raw[b].y - centre.y, raw[b].x - centre.x);
        return aa < bb;
    });

    std::array<cv::Point2f, 4> ring {};
    for (int i = 0; i < 4; ++i) ring[i] = raw[order[i]];

    // 反向遍历 = 换绕向，用来消掉镜像解
    if (reverse) std::swap(ring[1], ring[3]);

    // 旋转环，让 ring[0] -> ring[1] 落在最长边（135mm 那条）上
    int longest = 0;
    float best = -1.0f;
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f d = ring[(i + 1) % 4] - ring[i];
        const float len = d.x * d.x + d.y * d.y;
        if (len > best)
        {
            best = len;
            longest = i;
        }
    }

    std::array<cv::Point2f, 4> out {};
    for (int i = 0; i < 4; ++i) out[i] = ring[(longest + i) % 4];
    return out;
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

    output.setZero();
    output_valid_ = false;
    proj_err = std::numeric_limits<double>::infinity();
    last_branch_ = -1;

    if (corners.size() != 4) return;

    std::array<cv::Point2f, 4> raw {};
    for (int i = 0; i < 4; ++i)
        raw[i] = cv::Point2f(static_cast<float>(corners[i].x),
                             static_cast<float>(corners[i].y));


    std::vector<Candidate> candidates;


    for (const bool reverse : {false, true})
    {
        const std::array<cv::Point2f, 4> ordered = canonicalize(raw, reverse);

        std::vector<cv::Mat> rvecs {}, tvecs {};
        const int nsol = cv::solvePnPGeneric(armorPoint, ordered,
            camera_matrix_, distortion_coeffs_, rvecs, tvecs,
            false, cv::SOLVEPNP_IPPE, cv::noArray());
        if (nsol <= 0) continue;

        for (int i = 0; i < nsol; ++i)
        {
            const double tz = tvecs[i].at<double>(2);
            if (!(tz > 0.0)) continue;

            cv::Mat R;
            cv::Rodrigues(rvecs[i], R);

            std::vector<cv::Point2d> reproj {};
            cv::projectPoints(armorPoint, rvecs[i], tvecs[i],
                              camera_matrix_, distortion_coeffs_, reproj);
            if (reproj.size() != 4) continue;

            double sq = 0.0;
            for (int k = 0; k < 4; ++k)
            {
                const cv::Point2d diff = reproj[k] - cv::Point2d(ordered[k]);
                sq += diff.dot(diff);
            }
            const double rms = std::sqrt(sq / 4.0);
            if (!std::isfinite(rms) || rms > config.max_reproj_err) continue;

            // 板法向 = 板坐标系的 x 轴
            const Eigen::Vector3d normal(R.at<double>(0, 0),
                                         R.at<double>(1, 0),
                                         R.at<double>(2, 0));
            const Eigen::Vector3d tvec_e(tvecs[i].at<double>(0),
                                         tvecs[i].at<double>(1),
                                         tvecs[i].at<double>(2));
            const double yaw = std::atan2(normal.x(), normal.z());

            double dyaw = 0.0;
            if (std::isfinite(previous_yaw)) dyaw = wrap(yaw - previous_yaw);

            Candidate c;
            c.rms = rms;
            c.cost = rms + config.kyaw * dyaw * dyaw;
            c.tz = tz;
            c.yaw = yaw;
            c.normal = normal;
            c.rvec = rvecs[i];
            c.tvec = tvecs[i];
            c.branch = i;
            c.inward = normal.dot(tvec_e) > 0.0;
            c.reverse = reverse;
            c.measurement << tvec_e.x() / 1000.0, tvec_e.y() / 1000.0,
                             tvec_e.z() / 1000.0, yaw;
            candidates.push_back(c);
        }
    }

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
        if (R.col(0).dot(tvecs[i]) > 0)continue;

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




        //std::cout << "sol " << i
        //  << " rms=" << rms
        //  << " n·t=" << R.col(0).dot(tvecs[i])
        //  << " yaw=" << yaw * 57.3 << '\n';
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