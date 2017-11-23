#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include "AprilTagDetector/AprilTagDetector.h"

int main(int argc, char* argv[]) {
    if(argc != 2) {
        cout << "Usage: extract_camera_pose_calibration_points <path to image file>" << endl;
        return 1;
    }

    cv::Mat cv_img = cv::imread( argv[1] );
    cvtColor(cv_img, cv_img, cv::COLOR_BGR2GRAY);

    AprilTagDetector detector(AprilTagFamily::Tag25h9);
    auto detections = detector.detect(cv_img);

    for(auto detection: detections) {
        auto point = detection.points[AprilTagDetection::i_bottom_left];
        auto id = detection.id;
        int x = id % 4;
        int y = (id / 4) % 4;
        int z = (id / 16);
        cout << "(" << point[0] << "," << point[1] << "),"
             << "(" << x << "," << y << "," << z << ")" << endl;
    }
}