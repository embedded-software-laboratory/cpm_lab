#include "DetectionDispatcherLogic.h"


tuple<vector<cv::Rect>, bool> DetectionDispatcherLogic::apply(
    vector<tuple<time_point, AprilTagDetection>> previous_detections,
    vector<cpm_msgs::VehicleState> vehicle_states
) {
    return tuple<vector<cv::Rect>, bool>{vector<cv::Rect>{},false};
}
