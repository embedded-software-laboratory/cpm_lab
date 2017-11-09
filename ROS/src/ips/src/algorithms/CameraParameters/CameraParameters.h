#pragma once

#include <opencv2/opencv.hpp>
#include "utils/Parameter.h"

//! Stores camera parameters as described in <a href="https://docs.opencv.org/3.3.0/d9/d0c/group__calib3d.html">Camera Calibration and 3D Reconstruction</a>.
struct CameraParameters {
    //@{
    /*! Extrinsic camera parameter as described in <a href="https://docs.opencv.org/3.3.0/d9/d0c/group__calib3d.html">Camera Calibration and 3D Reconstruction</a>. */
    Parameter<cv::Matx33d> R = Parameter<cv::Matx33d>("R");
    Parameter<cv::Vec3d> T = Parameter<cv::Vec3d>("T");
    //@}

    //@{
    /*! Intrinsic camera parameter as described in <a href="https://docs.opencv.org/3.3.0/d9/d0c/group__calib3d.html">Camera Calibration and 3D Reconstruction</a>. */
    Parameter<double> fx = Parameter<double>("fx");
    Parameter<double> fy = Parameter<double>("fy");
    Parameter<double> cx = Parameter<double>("cx");
    Parameter<double> cy = Parameter<double>("cy");
    Parameter<double> k1 = Parameter<double>("k1");
    Parameter<double> k2 = Parameter<double>("k2");
    Parameter<double> k3 = Parameter<double>("k3");
    Parameter<double> p1 = Parameter<double>("p1");
    Parameter<double> p2 = Parameter<double>("p2");
    //@}

    /*!
        Simulate the projection of the camera from 3D world coordinates to 2D pixel coordinates.
        \param objectPoints (1xN) or (Nx1) list of 3D vectors in world coordinates.
        \return (1xN) or (Nx1) list of 2D vectors in pixel coordinates.
    */
    cv::Mat2d project(const cv::Mat3d &objectPoints);

    /*!
        Calculate the 3D rays that corresponds to pixels.
        \param imagePoints (1xN) or (Nx1) list of 2D vectors in pixel coordinates.
        \return <tt>tuple(origin, directions)</tt>, 3D rays of the form <tt>ray(p) = origin + p * direction</tt>
        where \c direction is one of N entries in \c directions.
    */
    std::tuple<cv::Vec3d, cv::Mat3d> pixelRays(cv::Mat2d imagePoints);

    /*! Calculate the extrinsic parameters CameraParameters::R and CameraParameters::T from pairs of corresponding world and image points. */
    void setExtrinsicsFromPnP(std::vector<cv::Point3d> objPts, std::vector<cv::Point2d> imgPts);
};