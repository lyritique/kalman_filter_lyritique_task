# ifndef __ARMOR_DETECTOR__
# define __ARMOR_DETECTOR__

# include <opencv2/core/mat.hpp>
# include <string>
# include <vector>
# include <array>

struct DetectorConfig {
    std::string model_path;
    int input_width = 640;
    int input_height = 480;
    float score_threshold = 0.5f;
    float nms_threshold = 0.45f;
    int ignore_color = -1;
    bool ignore_purple = true;
};

//letterbox变换的参数，记下以便处理后复原
struct LetterboxInfo
{
    float scale = 1.0f;   // 等比缩放系数
    float pad_x = 0.0f;   // 左侧填充的像素数
    float pad_y = 0.0f;   // 顶部填充的像素数
};

struct ArmorDetection {
    std::array<cv::Point2f, 4> corners; // LT, LB, RB, RT
    cv::Rect2f box;
    float score = 0.0f;
    int color = -1;
    int number = -1;
    bool big = false;
};



using ArmorDetections = std::vector<ArmorDetection>;

class Detector {
public:
    explicit Detector(const DetectorConfig& config); // 构造时加载模型
    ArmorDetections detect(const cv::Mat& bgr);
    const DetectorConfig& config() const;

private:
    struct Blob {
        cv::Mat blob;
        LetterboxInfo info;
    };

    Blob makeBlob(const cv::Mat& bgr) const;
    void decode(const cv::Mat& output, const Blob& input,
                int image_width, int image_height,
                ArmorDetections& detections) const;
    static void nms(ArmorDetections& detections, float iou_threshold);

    DetectorConfig config_;
    cv::dnn::Net net_;
};














# endif