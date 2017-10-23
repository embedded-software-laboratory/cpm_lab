
#include <stdio.h>
#include <stdint.h>
#include "apriltag/apriltag.h"
#include "opencv2/opencv.hpp"
#include "AprilTagDetector.h"
#include "Parameter.h"

#include <iostream>


struct CameraParameters {
    Parameter<cv::Matx33d> R;
    Parameter<cv::Vec3d> T;
    Parameter<double> fx, fy, cx, cy, k1, k2, k3, p1, p2;



    cv::Point2d reproject(cv::Point3d point) {
        cv::Matx31d p = R() * cv::Matx31d(point.x, point.y, point.z) + T();

        double x = p(0, 0);
        double y = p(1, 0);
        double z = p(2, 0);
        assert(z > 0);
        x /= z;
        y /= z;

        double x_sq = x * x;
        double y_sq = y * y;
        double r_sq = x_sq + y_sq;
        double r4 = r_sq * r_sq;
        double r6 = r4 * r_sq;

        double x2 = x * (1 + k1 * r_sq + k2 * r4 + k3 * r6) + 2 * p1 * x * y + p2 * (r_sq + 2 * x_sq);
        double y2 = y * (1 + k1 * r_sq + k2 * r4 + k3 * r6) + p1 * (r_sq + 2 * y_sq) + 2 * p2 * x * y;
        double u = fx * x2 + cx;
        double v = fy * y2 + cy;

        return cv::Point2d(u,v);
    }


    void setExtrinsicsFromPnP(std::vector<cv::Point3d> objPts, std::vector<cv::Point2d> imgPts) {
        cv::Vec3d rvec,t;
        cv::Matx33d r;
        cv::Matx33d cameraMatrix(
                fx(), 0, cx(),
                0, fy(), cy(),
                0,  0,  1);
        cv::Vec4d distParam(k1(), k2(), p1(), p2());
        cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, t);
        cv::Rodrigues(rvec, r);
        T = t;
        R = r;
    }


};



void calculateRelativeTransform(std::vector<AprilTagDetection> detections, CameraParameters &params) {
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


    // Manually labelled
    objPts.push_back(cv::Point3d(3,3,0));
    imgPts.push_back(cv::Point2d(993,223));

    objPts.push_back(cv::Point3d(0,3,0));
    imgPts.push_back(cv::Point2d(348,158));

    objPts.push_back(cv::Point3d(3,0,0));
    imgPts.push_back(cv::Point2d(1482,638));


    params.setExtrinsicsFromPnP(objPts, imgPts);

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

        CameraParameters params;

        params.fx = 945.444618;
        params.fy = 898.791897;
        params.cx = 776.656112;
        params.cy = 533.951952;
        params.k1 = -0.253795;
        params.k2 = 0.041498;
        params.k3 = 0;
        params.p1 = 0.019698;
        params.p2 = 0.001308;

        calculateRelativeTransform(detections, params);


        cvtColor(cv_img, cv_img, cv::COLOR_GRAY2BGR);



        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 3; ++y) {
                cv::line(cv_img, params.reproject(cv::Point3d(x,y + .2,0)), params.reproject(cv::Point3d(x,y + .8,0)), cv::Scalar(0,255,0),4);
                cv::line(cv_img, params.reproject(cv::Point3d(y + .2,x,0)), params.reproject(cv::Point3d(y + .8,x,0)), cv::Scalar(0,0,255),4);

            }
        }



        for (int x = 0; x < 4; ++x) {
            for (int y = 0; y < 4; ++y) {
                auto pt1 = params.reproject(cv::Point3d(x,y,0));
                auto pt2 = params.reproject(cv::Point3d(x,y,0.4));
                cv::line(cv_img, pt1, pt2, cv::Scalar(255,0,0),4);
            }
        }
        cv::namedWindow("a");
        cv::imwrite("CameraCalibrationDemo.png", cv_img);
        cv::imshow("a",cv_img);
        cv::waitKey(0);
    }

    return 0;
}
