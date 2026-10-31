# include "kalman_filter_task.hpp"
# include "armor_detector.hpp"
# include "imu_alignment.hpp"
# include "data_interaction.hpp"

# include <opencv2/videoio.hpp>
# include <string_view>
# include <fstream>

namespace {

}


int main(int argc, char** argv)
{
    
    //检测命令行输入,理论是 ./kalman_filter [video] [imu_txt] [output_dir]
    //设置默认值
    std::string imu_path = "material/2026-05-20_23-57-31.txt";
    std::string output_path = "output";
    std::string video_path = "material/2026-05-20_23-57-31 (1).avi";

    {
    //输入的参数覆盖默认值
    std::vector<std::string> args(argv + 1, argv + argc);
    //需要检测输入字符串末尾是否有对应后缀
    constexpr std::string_view formEnds[] {".mp4",".avi",".mkv"};

    if (args[1].ends_with(".txt"&& argc > 3))imu_path = args[1];
    if (argc > 2 ) for (auto formEnd :formEnds) if (args[0].ends_with(formEnd))video_path = args[0];
    //千层饼。先是有这个参数，再将其依次与视频后缀做判断
    if (argc > 4) output_path = args[3];
    }
    

    //几个部件全都构建初始化了
    auto imu_data = kalfilter::readImuTxt(imu_path);//内涵错误报错
    detector::Detector detector;
    pnp::pnpSolver pnp;
    kalfilter::KalmanFilter ekf(kalfilter::KalmanConfig config);//使用默认值

    //video的读写
    cv::VideoCapture cap(video_path);//写这个就要小心了
    if (!cap.isOpened()) throw std::runtime_error("读不了视频文件: " + video_path);
    //获得视频相关信息fps与总帧数
    double fps {cap.get(cv::CAP_PROP_FPS)};
    double frame_number {cap.get(cv::CAP_PROP_FRAME_COUNT)};
    //读第一帧，拿cv::size
    cv::Mat frame;
    cap >> frame;
    if (frame.empty())  throw std::runtime_error("第一帧读不了。 " + video_path) ;
    cv::Size frameSize = frame.size(); 
    int width  = frame.cols;
    int height = frame.rows;
    cv::VideoWriter writer("output/output_video.avi", 
        cv::VideoWriter::fourcc('m','p','4','v'),fps, frameSize);
    
    //CSV的创建
    std::ofstream csv ("output/data.csv");
    if (!csv) throw std::runtime_error("csv无法创建。");
    csv << "     \r\n";
    
    //
    double previous_image_time = 0.0;
    bool has_previous_time = false;

    /*
    for (int frame_index = 0; cv::readOneFrame(cap, frame); ++frame_index) {
        const double image_time = timelineTime(frame_index);
        const auto camera_imu =interpolateQuat(imu_data, image_time - 0.009);

        auto detections = detector.detect(frame);

        // 1. 选择本车目标，再对候选装甲板做 PnP
        // 2. 只保留 valid、类别/颜色一致、离预测位置合理的观测
        auto measurements = detectionsToMeasurements(detections, pnp);

        if (ekf.initialized()) {
            ekf.predict(image_time - previous_image_time, config);

            // 每帧只 predict 一次；多个可见板可以在 predict 后依次 update
            updateWithAssociatedMeasurements(ekf, measurements, camera_imu, config);
            ekf.renewArmorList();
        } else {
            // 连续若干帧 PnP 稳定后再 init，避免单帧错误初始化
            if (measurementsAreStable(measurements)) {
                ekf.init(measurements.front().armor);
            }
        }

        const auto state = ekf.getOutput(); // 需要新增只读接口
        logToCsv(image_time, state, ekf.covariance());

        drawDetections(frame, detections);
        drawFourArmorReprojections(frame, state, camera_imu);
        drawStateText(frame, state);
        writer.write(frame);

        previous_image_time = image_time;
        has_previous_time = true;
    }

    */
}