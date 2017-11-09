#include "tools/unittest/catch.hpp"
#include "AprilTagDetector.h"
#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include <ros/package.h>


cv::Mat1b loadImg(std::string file) {
    auto filename = ros::package::getPath("ips") + "/src/algorithms/AprilTagDetector/" + file;
    cv::Mat img = cv::imread( filename );
    cv::Mat1b img_bw;
    cvtColor(img, img_bw, cv::COLOR_BGR2GRAY);
    return img_bw;
}


TEST_CASE("AprilTagDetector_simple_examples") {
    SECTION( "Image test_img_tag25h9_ids_0_4" ) {
        AprilTagDetector detector(AprilTagFamily::Tag25h9);
        auto detections = detector.detect(loadImg("test_img_tag25h9_ids_0_4.jpg"));

        REQUIRE(detections.size() == 2);
        CHECK(((detections[0].id == 0 && detections[1].id == 4)
            ||(detections[0].id == 4 && detections[1].id == 0)));
    }

    SECTION( "Image test_img_tag36h11_id_0" ) {
        AprilTagDetector detector(AprilTagFamily::Tag36h11);

        auto img = loadImg("test_img_tag36h11_id_0.jpg");
        vector<AprilTagDetection> detections;

        SECTION("Normal test") {
            detections = detector.detect(img);
        }

        SECTION("With ROI crop") {
            cv::Rect ROI(299, 208, 128, 100);
            detections = detector.detect(img(ROI), ROI.tl());
        }

        SECTION("With ROI crop clone") {
            cv::Rect ROI(299, 208, 128, 100);
            detections = detector.detect(img(ROI).clone(), ROI.tl());
        }

        REQUIRE(detections.size() == 1);
        CHECK(detections[0].id == 0);

        CHECK(fabs(detections[0].points[AprilTagDetection::i_center][0] - 357) < 1);
        CHECK(fabs(detections[0].points[AprilTagDetection::i_center][1] - 257) < 1);

        CHECK(fabs(detections[0].points[AprilTagDetection::i_top_right][0] - 380) < 1);
        CHECK(fabs(detections[0].points[AprilTagDetection::i_top_right][1] - 271) < 1);
    }
}

