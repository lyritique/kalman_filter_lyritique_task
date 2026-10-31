# include "draw.hpp"

# include <array>
# include <cmath>
# include <iomanip>
# include <sstream>
# include <string>
# include <vector>

# include <opencv2/calib3d.hpp>
# include <opencv2/imgproc.hpp>









namespace draw {

namespace  {
cv::Scalar getcolor(int color)
{
    switch (color)
    {
    case 0: return {0, 165, 255};      //蓝对应橙
    case 1: return {255, 255, 255};    //红对应白
    case 2: return {0, 0, 0};          //白对应黑
    case 3: return {0, 255, 0};        //紫对应绿
    default: return {0, 255, 0};
    }
}
}



//detector
void drawDetections(
    cv::Mat& frame,
    const detector::ArmorDetections& detections)
{
    //每个装甲板依次
    for (const auto& detection : detections)
    {
        //构造舍入后的整数点
        std::vector<cv::Point> points;
        points.reserve(4);

        for (const cv::Point2f& corner : detection.corners)
        {
            points.emplace_back(
                cvRound(corner.x),
                cvRound(corner.y));
        }

        //画外轮廓
        cv::polylines(
            frame, points, true, getcolor(detection.color), 2);

        //第一个点，左上
        const cv::Point origin = points.front();

        std::ostringstream label;
        label << detection.number << " "
              << std::fixed << std::setprecision(2)
              << detection.score;

        cv::putText(
            frame,
            label.str(),
            {origin.x, std::max(15, origin.y - 5)},
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            getcolor(detection.color),
            1,
            cv::LINE_AA);//抗锯齿
    }
}


//右上角ekf信息
void drawStateText(
    cv::Mat& frame,
    const kalfilter::KFResult& state)
{
    //准备5行文字
    std::array<std::string, 5> lines;

    if (!state.valid)
    {
        lines = {
            "EKF: not initialized",
            "",
            "",
            "",
            ""
        };
    }
    else//输入九维状态
    {
        std::ostringstream                             position;
        position << std::fixed << std::setprecision(2)
                 << "pos: "
                 << state.state(0) << ", "
                 << state.state(1) << ", "
                 << state.state(2);

        std::ostringstream                            velocity;
        velocity << std::fixed << std::setprecision(2)
                 << "vel: "
                 << state.state(4) << ", "
                 << state.state(5) << ", "
                 << state.state(6);

        std::ostringstream                             yaw;
        yaw << std::fixed << std::setprecision(2)
            << "yaw: " << state.state(3);

        std::ostringstream yaw_rate;
        yaw_rate << std::fixed << std::setprecision(2)
                 << "vyaw: " << state.state(7);

        std::ostringstream                             radius;
        radius << std::fixed << std::setprecision(3)
               << "r: " << state.state(8);

        lines = {
            position.str(),
            velocity.str(),
            yaw.str(),
            yaw_rate.str(),
            radius.str()
        };
    }

    constexpr int line_height {22};
    constexpr int margin  {8};

    //计算每行长度，取最长
    int text_width = 0;
    for (const auto& line : lines)
    {
        int baseline = 0;
        const cv::Size size = cv::getTextSize(
            line,
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            1,
            &baseline);

        text_width = std::max(text_width, size.width);
    }

    const int panel_width = text_width + 2 * margin;
    const int panel_height =
        static_cast<int>(lines.size()) * line_height + margin;

    //构造矩形背景（黑色）
    const cv::Rect back(
        std::max(0, frame.cols - panel_width),
        0,
        std::min(panel_width, frame.cols),
        std::min(panel_height, frame.rows));

    cv::rectangle(frame, back, cv::Scalar(0, 0, 0), cv::FILLED);

    //写字
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        cv::putText(
            frame,
            lines[i],
            {back.x + margin,
             back.y + margin + static_cast<int>(i) * line_height},
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA);
    }
}









}
