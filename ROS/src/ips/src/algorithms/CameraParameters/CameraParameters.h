#pragma once

#include <opencv2/opencv.hpp>
#include "utils/Parameter.h"

//! Stores camera parameters as described in <a href="https://docs.opencv.org/3.3.0/d9/d0c/group__calib3d.html">Camera Calibration and 3D Reconstruction</a>.
struct CameraParameters {
    //@{
    /*! Camera parameter as described in <a href="https://docs.opencv.org/3.3.0/d9/d0c/group__calib3d.html">Camera Calibration and 3D Reconstruction</a>. */
    Parameter<cv::Matx33d> R;
    Parameter<cv::Vec3d> T;
    Parameter<double> fx, fy, cx, cy, k1, k2, k3, p1, p2;
    //@}

    /*! Simulate the projection of the camera from 3D world coordinates to 2D pixel coordinates. */
    cv::Point2d project(cv::Point3d point);

    cv::Point3d ray(cv::Point2d);

    /*! Calculate the extrinsic parameters CameraParameters::R and CameraParameters::T from pairs of corresponding world an image points. */
    void setExtrinsicsFromPnP(std::vector<cv::Point3d> objPts, std::vector<cv::Point2d> imgPts);
};