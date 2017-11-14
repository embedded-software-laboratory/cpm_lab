#pragma once

#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include <cpm_msgs/VehicleState.h>
#include "algorithms/CameraParameters/CameraParameters.h"
#include "algorithms/AprilTagDetector/AprilTagDetector.h"

using AprilTagDetectionStamped = WithTimestamp<AprilTagDetection>;


class DetectionDispatcherLogic {
    CameraParameters cameraParameters;
public:
    DetectionDispatcherLogic(const CameraParameters &cameraParameters): cameraParameters(cameraParameters){}

    tuple<vector<cv::Rect>, bool, vector<optional<AprilTagDetectionStamped>>> apply(
        const vector<AprilTagDetectionStamped> &previous_detections,
        const vector<cpm_msgs::VehicleState> &vehicle_states
    );
};