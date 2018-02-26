#pragma once

#include "cpm_tools/default.hpp"
#include <opencv2/opencv.hpp>
#include "CameraParameters/CameraParameters.hpp"
#include "AprilTagDetector/AprilTagDetector.hpp"
#include "cpm_msgs/msg/vehicle_state.hpp"


template <typename Base>
struct WithTimestamp : public Base {
    uint64_t timestamp;
    WithTimestamp(){}
    WithTimestamp(const Base &base, uint64_t timestamp):Base(base),timestamp(timestamp){}
    WithTimestamp(Base &&base, uint64_t timestamp):Base(std::move(base)),timestamp(timestamp){}
};

using AprilTagDetectionStamped = WithTimestamp<AprilTagDetection>;

//! Decision logic for selective detection on image crops
class DetectionDispatcherLogic {
    CameraParameters cameraParameters;
public:
    DetectionDispatcherLogic(const CameraParameters &cameraParameters): cameraParameters(cameraParameters){}

    //! Decide in which parts of the image the detection should be applied, depending on previous detections and vehcile states.
    //! \return <tt>tuple(ROIs, full_frame_detection, detection_per_vehicle)</tt><br/>
    //! <tt>ROIs</tt> List of ROIs for detection in the next frame.<br/>
    //! <tt>full_frame_detection</tt> Whether a detection should be run on the entire image.<br/>
    //! <tt>detection_per_vehicle</tt> Has a 1-to-1 correspondence with \c vehicle_states, ids are equal. Subset of \c previous_detections.
    tuple<vector<cv::Rect>, bool, vector<optional<AprilTagDetectionStamped>>> apply(
        const vector<AprilTagDetectionStamped> &previous_detections,
        const vector<cpm_msgs::msg::VehicleState> &vehicle_states
    );
};