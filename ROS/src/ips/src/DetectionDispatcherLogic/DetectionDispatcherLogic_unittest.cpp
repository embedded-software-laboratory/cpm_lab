#include "DetectionDispatcherLogic.h"
#include "tools/unittest/catch.hpp"
#include "utils/default.h"


TEST_CASE("DetectionDispatcherLogic") {

    CameraParameters params;

    params.fx = 1;
    params.fy = 1;
    params.cx = 0;
    params.cy = 0;
    params.k1 = 0;
    params.k2 = 0;
    params.k3 = 0;
    params.p1 = 0;
    params.p2 = 0;
    params.R = cv::Matx33d(1,0,0,0,1,0,0,0,1);
    params.T = cv::Vec3d(0,0,0);


    DetectionDispatcherLogic detectionDispatcherLogic(params);


    SECTION("Single vehicle") {

        AprilTagDetection aprilTagDetection;
        aprilTagDetection.id = 42;
        aprilTagDetection.points[AprilTagDetection::i_center]       = {2,2};
        aprilTagDetection.points[AprilTagDetection::i_bottom_left]  = {1,1};
        aprilTagDetection.points[AprilTagDetection::i_bottom_right] = {3,1};
        aprilTagDetection.points[AprilTagDetection::i_top_left]     = {1,3};
        aprilTagDetection.points[AprilTagDetection::i_top_right]    = {3,3};
        WithTimestamp<AprilTagDetection> aprilTagDetectionStamped(aprilTagDetection, ros::Time().fromNSec(12345678987654321));

        cpm_msgs::VehicleState vehicle_state;
        vehicle_state.id = 42;
        vehicle_state.pose.position.x = -5;
        vehicle_state.pose.position.y = -5;
        vehicle_state.pose.position.z = 1;

        SECTION("Position known, detection known") {
            // Typical case, the vehicle knows its position and receives regular position updates

            vector<cv::Rect> ROIs;
            bool full_frame_detection;
            vector<optional<AprilTagDetectionStamped>> detection_per_vehicle;
            tie(ROIs, full_frame_detection, detection_per_vehicle) = detectionDispatcherLogic.apply( {aprilTagDetectionStamped}, {vehicle_state} );

            CHECK(!full_frame_detection);
            CHECK(ROIs.size() == 1);
        }

        SECTION("Position unknown, detection known") {
            // The vehicle is detected for the first time

            vehicle_state.pose.position.x = NaN;
            vehicle_state.pose.position.y = NaN;
            vehicle_state.pose.position.z = NaN;

            vector<cv::Rect> ROIs;
            bool full_frame_detection;
            vector<optional<AprilTagDetectionStamped>> detection_per_vehicle;
            tie(ROIs, full_frame_detection, detection_per_vehicle) = detectionDispatcherLogic.apply( {aprilTagDetectionStamped}, {vehicle_state} );

            CHECK(!full_frame_detection);
            CHECK(ROIs.size() == 1);
        }

        SECTION("Position known, detection unknown") {
            // The vehicle is out of view but still knows its position, e.g. through internal sensors or other cameras.

            vector<cv::Rect> ROIs;
            bool full_frame_detection;
            vector<optional<AprilTagDetectionStamped>> detection_per_vehicle;
            tie(ROIs, full_frame_detection, detection_per_vehicle) = detectionDispatcherLogic.apply( {}, {vehicle_state} );

            CHECK(!full_frame_detection);
            CHECK(ROIs.size() == 1);
        }

        SECTION("Position unknown, detection unknown") {
            // No knowledge about the vehicle -> full frame detection necessary

            vehicle_state.pose.position.x = NaN;
            vehicle_state.pose.position.y = NaN;
            vehicle_state.pose.position.z = NaN;

            vector<cv::Rect> ROIs;
            bool full_frame_detection;
            vector<optional<AprilTagDetectionStamped>> detection_per_vehicle;
            tie(ROIs, full_frame_detection, detection_per_vehicle) = detectionDispatcherLogic.apply( {}, {vehicle_state} );

            CHECK(full_frame_detection);
            CHECK(ROIs.size() == 0);
        }
    }

}