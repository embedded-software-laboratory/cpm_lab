
#include <stdio.h>
#include <stdint.h>
#include "apriltag/apriltag.h"
#include "opencv2/opencv.hpp"
#include "AprilTagDetector.h"

#include <iostream>






void getRelativeTransform(std::vector<AprilTagDetection> detections, double fx, double fy, double px, double py, double k1, double k2, double p1, double p2) {
    std::vector<cv::Point3d> objPts;
    std::vector<cv::Point2d> imgPts;


    for(auto detection: detections) {
        int i = detection.id;
        int x = i % 4;
        int y = (i / 4) % 4;
        int z = (i / 16);
        objPts.push_back(cv::Point3d(x,y,z));
        imgPts.push_back(cv::Point2d(detection.points[AprilTagDetection::i_center][0], detection.points[AprilTagDetection::i_center][1]));
    }



    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix(
            fx, 0, px,
            0, fy, py,
            0,  0,  1);
    cv::Vec4d distParam(k1, k2, p1, p2);
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);

    std::cout << tvec << std::endl;
    /*Eigen::Matrix3d wRo;
    wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;

    return T;*/
}



int main(int argc, char *argv[])
{
    AprilTagDetector my_detector;

    for (int input = 0; input < 1; input++) {

        auto cv_img = cv::imread( "/home/janis/sciebo/CPM-Lab/04_Measurement/CameraCalibration/2017-10-18-Basler-dart-daA1600-60um/extrinsic/2017-10-18_16-26-55.242950.tif");
        cvtColor(cv_img, cv_img, cv::COLOR_BGR2GRAY);

        image_u8_t im = {.width = cv_img.cols,
                .height = cv_img.rows,
                .stride = cv_img.step.buf[0],
                .buf = cv_img.data
        };
        auto detections = my_detector.detect(im);
        for(auto det: detections) {
            printf(" id #%d,  hamming %d,  goodness %f,  margin %f\n",
                   det.id,
                   det.hamming,
                   det.goodness,
                   det.decision_margin
            );
        }

        double fx = 945.6139;
        double fy = 898.9172;
        double px = 770.7659;
        double py = 538.2835;
        double k1 = -0.2550;
        double k2 = 0.0593;
        double p1 = 0.0176;
        double p2 = 2.1927e-05;


        getRelativeTransform(detections, fx, fy, px, py, k1, k2, p1, p2);
    }

    return 0;
}
