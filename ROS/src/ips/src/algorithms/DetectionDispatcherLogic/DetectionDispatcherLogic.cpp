#include "DetectionDispatcherLogic.h"


tuple<vector<cv::Rect>, bool> DetectionDispatcherLogic::apply(
    vector<tuple<ros::Time, AprilTagDetection>> previous_detections,
    vector<cpm_msgs::VehicleState> vehicle_states
) {
    return make_tuple(vector<cv::Rect>{}, false);
}
