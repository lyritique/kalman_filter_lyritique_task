# ifndef __DRAW__
# define __DRAW__

# include "armor_detector.hpp"
# include "kalman_filter_task.hpp"

#include <opencv2/core.hpp>



namespace draw {




void drawDetections(
    cv::Mat& frame,
    const detector::ArmorDetections& detections);

void drawStateText(
    cv::Mat& frame,
    const kalfilter::KFResult& state);





 







}//namespce draw












# endif