# include "kalman_filter_task.hpp"
# include "armor_detector.hpp"
# include "imu_alignment.hpp"
# include "data_interaction.hpp"

# include <cmath>
# include <opencv2/videoio.hpp>
# include <string_view>
# include <fstream>
# include <filesystem>

namespace {
/*
void FrametoKFresult(const cv::Mat& frame,
detector::Detector& detector,pnp::pnpSolver& pnp, kalfilter::KalmanFilter& ekf)
{
    auto detections = detector.detect(frame);

        for (auto detection:detections)
        {
            //pnp
            pnp::PnPResult pose {};
            pnp.input(detection.corners, detection.big);
            pnp.easySolver();
            pose = pnp.getOutput();

            //这里写检测

            //ekf的update
            kalfilter::armerState efk_input{};
            efk_input.measurement = std::move(pose.measurement);
            
        }

}
*/

void computeYPD (Eigen::Matrix<double,4,1>& measurement,kalfilter::cam_ypd& ypd)//x,y,z,yaw
{
    /*
    azi  = atan2(x, z)
    ele  = atan2(y, sqrt(x² + z²))
    dist = sqrt(x² + y² + z²)
    */
    ypd.azi = std::atan2(measurement[0],measurement[2]);
    ypd.ele = std::atan2(measurement[1],
        std::hypot(measurement[0], measurement[2]));
    ypd.dist = std::sqrt(measurement[0]*measurement[0] +
        measurement[1]*measurement[1] +
        measurement[2]*measurement[2]);

}
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

    if (argc > 3 && args[1].ends_with(".txt"))imu_path = args[1];
    if (argc > 2 ) for (auto formEnd :formEnds) if (args[0].ends_with(formEnd))video_path = args[0];
    //千层饼。先是有这个参数，再将其依次与视频后缀做判断
    if (argc > 4) output_path = args[3];
    }
    
    //创建输出目录
    std::filesystem::create_directories(output_path);

    //几个部件全都构建初始化了
    auto imu_data = kalfilter::readImuTxt(imu_path);//内涵错误报错
    if (imu_data.empty()) throw std::runtime_error("IMU 数据为空");
    detector::Detector detector;
    pnp::pnpSolver pnp;
    kalfilter::KalmanFilter ekf;//都使用默认值

    //video的读写
    cv::VideoCapture cap(video_path);//写这个就要小心了
    if (!cap.isOpened()) throw std::runtime_error("读不了视频文件: " + video_path);
    //获得视频相关信息fps与总帧数
    double fps {cap.get(cv::CAP_PROP_FPS)};
    double frame_number {cap.get(cv::CAP_PROP_FRAME_COUNT)};
    if (fps <= 0 ||fps > 1000) throw std::runtime_error("视频有问题");
    //读第一帧，拿cv::size
    cv::Mat frame;
    cap >> frame;
    if (frame.empty())  throw std::runtime_error("第一帧读不了。 " + video_path) ;
    cv::Size frameSize = frame.size(); 
    int width  = frame.cols;
    int height = frame.rows;
    
    //第一帧拿都拿了，init一下ekf
    {
        auto detections = detector.detect(frame);
        Eigen::Matrix<double,4,1> best {};

        for (auto detection:detections)
        {
            //pnp
            pnp::PnPResult pose {};
            pnp.input(detection.corners, detection.big);
            pnp.easySolver();
            pose = pnp.getOutput();
        
            //这里写检测
            //这里的检测应该要挑出最好的
            best = {};
        }
            //ekf的input
            kalfilter::armerState efk_input{};
            efk_input.measurement = std::move(best);

            ekf.init(efk_input);
    }
    
    //用上面的数据构建输出video的writer
    cv::VideoWriter writer(output_path + "output_video.avi", 
        cv::VideoWriter::fourcc('M','J','P','G'),fps, frameSize);
    if (!writer.isOpened()) throw std::runtime_error("无法创建输出视频");

    
    //CSV的创建
    std::ofstream csv (output_path + "output/data.csv");
    if (!csv) throw std::runtime_error("csv无法创建。");
    csv << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r\r\n";
    
    //重设为第一帧
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);

    //相信imu的总时间（22.5s）还是视频的总时间（17s），是个问题
    //我认为1秒帧数不是固定的，转到视频时变成30FPS了
    //所有视频时间不可信，要重新计算
    const double video_start_time = imu_data.front().time;
    double previous_image_time {};
    bool has_previous_time = false;

    for (int frame_index {};frame_index < frame_number;++frame_index)
    {
        //处理时间
        const double image_time {video_start_time + frame_index / fps};
        const double dt {has_previous_time ? image_time-previous_image_time : image_time};
        previous_image_time = image_time;
        const kalfilter::imu cam_imu {kalfilter::alignImuToImage (imu_data,image_time)};

        ekf.predict(dt);

        cap.read(frame);
        auto detections = detector.detect(frame);
        kalfilter::cam_ypd ypd {};

        for (auto detection:detections)
        {
            //pnp
            pnp::PnPResult pose {};
            pnp.input(detection.corners, detection.big);
            pnp.easySolver();
            pose = pnp.getOutput();
            computeYPD(pose.measurement, ypd);

            //这里写检测

            //这里draw装甲板

            //ekf的update
            kalfilter::armerState efk_input{};
            efk_input.measurement = std::move(pose.measurement);
            ekf.update(efk_input, ypd);
        }
        ekf.renewArmorList();

        auto output { ekf.getOutput()};

        //csv输入
        if (output.valid)csv << image_time << "偏离过大";//多次偏离应该reset，但还未写
        else csv << image_time << output.state;

        //这里draw数据

    }//循环结束

    //收尾





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