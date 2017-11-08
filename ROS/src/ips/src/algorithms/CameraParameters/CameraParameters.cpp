#include "CameraParameters.h"

cv::Mat2d CameraParameters::project(const cv::Mat3d &objectPoints) {
    cv::Matx33d cameraMatrix(
        fx(), 0, cx(),
        0, fy(), cy(),
        0,  0,  1);
    cv::Vec<double, 5> distCoeffs(k1(), k2(), p1(), p2(), k3());
    cv::Vec3d rvec;
    cv::Rodrigues(R(), rvec);
    cv::Mat imagePoints;
    cv::projectPoints(objectPoints, rvec, T(), cameraMatrix, distCoeffs, imagePoints);
    assert(imagePoints.type() == CV_64FC2);
    return cv::Mat2d(imagePoints);
}

std::tuple<cv::Vec3d, cv::Mat3d> CameraParameters::pixelRays(cv::Mat2d imagePoints) {

    cv::Matx33d cameraMatrix(
            fx(), 0, cx(),
            0, fy(), cy(),
            0,  0,  1);
    cv::Vec<double, 5> distCoeffs(k1(), k2(), p1(), p2(), k3());


    cv::Mat undistored;
    cv::undistortPoints(imagePoints, undistored, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::TermCriteria(cv::TermCriteria::COUNT,20,0.05));

    assert(undistored.type() == CV_64FC2);

    const int rows = undistored.rows;
    const int cols = undistored.cols;

    // unpack channels: each row = (xi,yi)
    undistored = undistored.reshape(1, rows*cols);

    assert(undistored.rows == rows*cols);
    assert(undistored.cols == 2);
    assert(undistored.type() == CV_64FC1);

    // add homogeneous z=1, each row = (xi,yi,zi) = (xi,yi, 1)
    cv::hconcat(undistored, cv::Mat::ones(rows*cols, 1, CV_64FC1), undistored);

    assert(undistored.rows == rows*cols);
    assert(undistored.cols == 3);
    assert(undistored.type() == CV_64FC1);

    // subtract offset T
    cv::subtract(undistored, cv::Mat(rows*cols, 1, CV_64FC3, cv::Scalar(T()[0], T()[1], T()[2])).reshape(1, rows*cols), undistored);

    assert(undistored.rows == rows*cols);
    assert(undistored.cols == 3);
    assert(undistored.type() == CV_64FC1);

    // apply inverse rotation
    undistored = undistored * cv::Mat(R());

    assert(undistored.rows == rows*cols);
    assert(undistored.cols == 3);
    assert(undistored.type() == CV_64FC1);

    undistored = undistored.reshape(3, rows);

    assert(undistored.rows == rows);
    assert(undistored.cols == cols);
    assert(undistored.type() == CV_64FC3);

    cv::Vec3d origin = R().t() * (-T());
    cv::Mat3d directions = cv::Mat3d(undistored) - origin;

    return std::make_tuple(origin, directions);
}

void CameraParameters::setExtrinsicsFromPnP(std::vector<cv::Point3d> objPts, std::vector<cv::Point2d> imgPts) {
    cv::Vec3d rvec,t;
    cv::Matx33d r;
    cv::Matx33d cameraMatrix(
            fx(), 0, cx(),
            0, fy(), cy(),
            0,  0,  1);
    cv::Vec<double, 5> distParam(k1(), k2(), p1(), p2(), k3());
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, t);
    cv::Rodrigues(rvec, r);
    T = t;
    R = r;
}