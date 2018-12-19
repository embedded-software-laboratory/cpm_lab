#include "default.hpp"


struct Cluster {
    float x_sum = 0;
    float y_sum = 0;
    float weight = 0;
};

vector<cv::Point2f> detect_light_blobs(cv::Mat img);