#include "DetectionDispatcherLogic.hpp"


tuple<vector<cv::Rect>, bool, vector<optional<AprilTagDetectionStamped>>> DetectionDispatcherLogic::apply(
    const vector<AprilTagDetectionStamped> &previous_detections,
    const vector<cpm_msgs::msg::VehicleState> &vehicle_states
) {
    // Create an index of vehicle IDs
    map<int, size_t> vehicle_id_map;
    for (size_t i = 0; i < vehicle_states.size(); ++i) vehicle_id_map[vehicle_states[i].id] = i;

    // Associate detections with vehicles, eliminate duplicate detections per vehicle.
    vector<optional<AprilTagDetectionStamped>> detection_per_vehicle(
        vehicle_states.size(),
        optional<AprilTagDetectionStamped>() // create a missing (optional) detection for each vehicle
    );
    for (auto const &detection : previous_detections) {
        auto search = vehicle_id_map.find(detection.id);
        if(search != vehicle_id_map.end()) {
            // detection is of a known vehicle

            const size_t vehicle_index = search->second;
            if(detection_per_vehicle[vehicle_index]) {
                // we have a duplicate detection, need to decide which to keep
                auto timestamp_old = detection_per_vehicle[vehicle_index]->timestamp;
                auto timestamp_new = detection.timestamp;
                if(timestamp_new > timestamp_old) {
                    // only replace the detection if it is more recent.
                    detection_per_vehicle[vehicle_index] = detection;
                }
            }
            else {
                // we have the first detection for this vehicle, so just store it.
                detection_per_vehicle[vehicle_index] = detection;
            }
        }
    }


    vector<cv::Rect> ROIs;
    bool full_frame_detection = false;

    // Determine the ROI for each vehicle.
    // Also, if we have no information for a vehicle, set the 'full_frame_detection' flag.
    for (size_t i = 0; i < vehicle_states.size(); ++i) {
        const bool detection_available = bool(detection_per_vehicle[i]);
        const bool position_available = (
                   !std::isnan(vehicle_states[i].pose.position.x)
                && !std::isnan(vehicle_states[i].pose.position.y)
                && !std::isnan(vehicle_states[i].pose.position.z)
        );
        if( !detection_available && !position_available ) {
            // We have no info about this vehicle, need to do a full_frame_detection.
            full_frame_detection = true;
        }
        else if (detection_available) {
            // Crop with the previous detection at the center.
            auto center = detection_per_vehicle[i]->points[AprilTagDetection::i_center];
            constexpr int crop_size = 200;
            ROIs.emplace_back(
                    int(center[0]) - crop_size/2,
                    int(center[1]) - crop_size/2,
                    crop_size,
                    crop_size
            );
        }
        else {
            // Crop with the image projection of the vehicle position at the center.
            cv::Mat3d vehicle_position(1, 1, cv::Vec3d(
                    vehicle_states[i].pose.position.x,
                    vehicle_states[i].pose.position.y,
                    vehicle_states[i].pose.position.z
            ));
            cv::Vec2d vehicle_image_position = cameraParameters.project(vehicle_position)(0,0);
            constexpr int crop_size = 300;
            ROIs.emplace_back(
                    int(vehicle_image_position[0]) - crop_size/2,
                    int(vehicle_image_position[1]) - crop_size/2,
                    crop_size,
                    crop_size
            );
        }

    }

    return make_tuple(ROIs, full_frame_detection, detection_per_vehicle);
}
