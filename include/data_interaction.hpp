# ifndef __DATA_INTERACTION__
# define __DATA_INTERACTION__

# include <array>
# include <vector>


# include <opencv2/core.hpp>
# include <opencv2/core/types.hpp>
# include <eigen3/Eigen/Eigen>


namespace pnp {

namespace  {


struct pnpConfig
{
    //小装甲板尺寸
    static constexpr double half_small_height {140 * 0.5};
    static constexpr double half_small_width  {125 * 0.5};

    std::vector<cv::Point3d> small_armorPoint 
    {
        cv::Point3f(0.0f, -half_small_width,  half_small_height),          // 左灯条下
        cv::Point3f(0.0f, -half_small_width, -half_small_height),          // 左灯条上
        cv::Point3f(0.0f,  half_small_width, -half_small_height),          // 右灯条上
        cv::Point3f(0.0f,  half_small_width,  half_small_height),          // 右灯条下
    };

    //大装甲板尺寸
    static constexpr double half_big_height {235 * 0.5};
    static constexpr double half_big_width  {127 * 0.5};

    std::vector<cv::Point3d> big_armorPoint 
    {
        cv::Point3f(0.0f, -half_big_width,  half_big_height),          // 左灯条下
        cv::Point3f(0.0f, -half_big_width, -half_big_height),          // 左灯条上
        cv::Point3f(0.0f,  half_big_width, -half_big_height),          // 右灯条上
        cv::Point3f(0.0f,  half_big_width,  half_big_height),          // 右灯条下
    };

    //video内参   
    struct camParams 
    {
    double width = 1440;
    double height= 1080;
    double fx    = 2362.2795952460974;
    double fy    = 2336.1916035376717;
    double cx    = 679.17513921636282;
    double cy    = 529.06858905837635;
    Eigen::Matrix<double, 5, 1> dist  {-0.098561592108101559, 0.11173066531932681,
           -0.0038352945867108721, -0.0020421443354436304, 0.0};
    };

    static const constexpr double max_reproj_err {3.0};                                                     //戴帽子大师



};
    
}





struct PnPResult
{
    bool valid = false;
    Eigen::Matrix<double, 4, 1> measurement {};
};





class pnpSolver
{
    public:
    //构造函数（未写）
    explicit pnpSolver();
    void input (const std::array<cv::Point2f, 4>& image_corners, bool big);
    void easySolver();
    PnPResult getOutput();
    private:

    Eigen::Matrix<double,4,1> output = Eigen::Matrix<double, 4, 1>::Zero();
    bool output_valid_ = false;
    std::vector<cv::Point2d> corners {};

    pnpConfig config {};
    //存装甲板系下的装甲板，构造默认小的
    std::vector<cv::Point3d> armorPoint {config.small_armorPoint};

    cv::Mat camera_matrix_{};
    cv::Mat distortion_coeffs_;

};

}//namespace pnp
# endif