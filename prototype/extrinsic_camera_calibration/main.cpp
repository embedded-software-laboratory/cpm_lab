
#include <stdio.h>
#include <stdint.h>
#include "apriltag/apriltag.h"
#include "opencv2/opencv.hpp"
#include "AprilTagDetector.h"

#include <iostream>




cv::Point2d reproject(cv::Point3d point, cv::Matx33d CameraR, cv::Vec3d CameraT, double fx, double fy, double px, double py, double k1, double k2, double p1, double p2) {


    cv::Matx31d p = CameraR * cv::Matx31d(point.x, point.y, point.z) + CameraT;


    double x = p(0, 0);
    double y = p(1, 0);
    double z = p(2, 0);
    assert(z > 0);
    x /= z;
    y /= z;
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double x2 = x * (1 + k1 * r2 + k2 * r4) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    double y2 = y * (1 + k1 * r2 + k2 * r4) + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    double u = fx * x2 + px;
    double v = fy * y2 + py;

    return cv::Point2d(u,v);
}





void getRelativeTransform(cv::Mat cv_img, std::vector<AprilTagDetection> detections, double fx, double fy, double px, double py, double k1, double k2, double p1, double p2) {
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


    objPts.push_back(cv::Point3d(3,3,0));
    imgPts.push_back(cv::Point2d(993,223));

    objPts.push_back(cv::Point3d(0,3,0));
    imgPts.push_back(cv::Point2d(348,158));

    objPts.push_back(cv::Point3d(3,0,0));
    imgPts.push_back(cv::Point2d(1482,638));

    /*objPts.push_back(cv::Point3d(0,0,0));
    imgPts.push_back(cv::Point2d(229,930));

    objPts.push_back(cv::Point3d(1,1,0));
    imgPts.push_back(cv::Point2d(781,422));*/




    cv::Vec3d rvec, tvec;
    cv::Matx33d cameraMatrix(
            fx, 0, px,
            0, fy, py,
            0,  0,  1);
    cv::Vec4d distParam(k1, k2, p1, p2);
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);

    std::cout << "r" << std::endl;
    std::cout << r << std::endl;
    std::cout << "t" << std::endl;
    std::cout << tvec << std::endl;

    cvtColor(cv_img, cv_img, cv::COLOR_GRAY2BGR);


    {
        // Reprojection
        std::cout << "Reprojection" << std::endl;
        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                auto pt1 = reproject(cv::Point3d(x,y,0), r, tvec, fx, fy, px, py, k1, k2, p1, p2);
                auto pt2 = reproject(cv::Point3d(x,y,0.4), r, tvec, fx, fy, px, py, k1, k2, p1, p2);

                //cv::circle(cv_img, pt_2d, 20, cv::Scalar(255), 3);
                cv::line(cv_img, pt1, pt2, cv::Scalar(0,0,255),4);
            }
        }
        cv::namedWindow("a");
        cv::imwrite("CameraCalibrationDemo.png", cv_img);
        cv::imshow("a",cv_img);
        cv::waitKey(0);

    }


}

int main(int argc, char *argv[])
{
    AprilTagDetector my_detector;

    for (int input = 0; input < 1; input++) {

        cv::Mat cv_img = cv::imread( "/home/janis/sciebo/CPM-Lab/04_Measurement/CameraCalibration/2017-10-18-Basler-dart-daA1600-60um/extrinsic/2017-10-18_16-26-55.242950.tif");
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

        double fx = 945.444618;
        double fy = 898.791897;
        double px = 776.656112;
        double py = 533.951952;
        double k1 = -0.253795;
        double k2 = 0.041498;
        double p1 = 0.019698;
        double p2 = 0.001308;


        getRelativeTransform(cv_img, detections, fx, fy, px, py, k1, k2, p1, p2);
    }

    return 0;
}
