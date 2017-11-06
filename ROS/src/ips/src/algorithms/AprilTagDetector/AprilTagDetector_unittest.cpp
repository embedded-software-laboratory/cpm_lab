#include "tools/unittest/catch.hpp"
#include "AprilTagDetector.h"
#include <opencv2/opencv.hpp>



TEST_CASE("AprilTagDetector_simple_example") {

    /***** Arrange *****/
    //auto filename = ros::package::getPath("ips") + "/src/algorithms/AprilTagDetector/test_img_tag25h9_ids_0_4.jpg";
    auto filename = "/home/janis/Dev/CPM-Lab/software/ROS/src/ips/src/algorithms/AprilTagDetector/test_img_tag25h9_ids_0_4.jpg";
    cv::Mat cv_img = cv::imread( filename );
    cvtColor(cv_img, cv_img, cv::COLOR_BGR2GRAY);
    image_u8_t im = {.width = cv_img.cols,
        .height = cv_img.rows,
        .stride = cv_img.step.buf[0],
        .buf = cv_img.data
    };
    AprilTagDetector detector(AprilTagFamily::Tag25h9);


    /***** Act *****/
    auto detections = detector.detect(im);

    /***** Assert *****/
    REQUIRE(detections.size() == 2);
    for(auto detection:detections) {
        CHECK((detection.id == 0 || detection.id == 4));
    }
}

