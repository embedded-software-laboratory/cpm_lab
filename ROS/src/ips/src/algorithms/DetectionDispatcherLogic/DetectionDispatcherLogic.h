#pragma once

#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include <cpm_msgs/VehicleState.h>
#include "algorithms/CameraParameters/CameraParameters.h"
#include "algorithms/AprilTagDetector/AprilTagDetector.h"

class DetectionDispatcherLogic {
    CameraParameters cameraParameters;
public:
    DetectionDispatcherLogic(const CameraParameters &cameraParameters): cameraParameters(cameraParameters){}

    tuple<vector<cv::Rect>, bool> apply(
        vector<tuple<ros::Time, AprilTagDetection>> previous_detections,
        vector<cpm_msgs::VehicleState> vehicle_states
    );
};