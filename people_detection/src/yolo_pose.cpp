// yolo_pose.cpp

#include "people_detection/main.h"  // Ajusta la inclusión según tu estructura

void yoloPose(cv::Mat &image, YoloPose &yolo) {
    auto result = yolo.detect(image);
    ImageTools::draw(result, image);
    ImageTools::show(image);
    cv::waitKey(1);
}
