#include "CameraParameters.h"

cv::Point2d CameraParameters::project(cv::Point3d point) {
    cv::Matx31d p = R() * cv::Matx31d(point.x, point.y, point.z) + T();
    assert(p(2, 0) > 0);

    cv::Matx33d cameraMatrix(
        fx(), 0, cx(),
        0, fy(), cy(),
        0,  0,  1);
    cv::Vec<double, 5> distCoeffs(k1(), k2(), p1(), p2(), k3());
    cv::Vec3d rvec;
    cv::Rodrigues(R(), rvec);
    cv::Mat objectPoints(1, 1, CV_64FC3, cv::Scalar(p(0), p(1), p(2)));
    cv::Mat imagePoints;
    cv::projectPoints(objectPoints, rvec, T(), cameraMatrix, distCoeffs, imagePoints);
    assert(imagePoints.type() == CV_64FC2);
    auto p2 = imagePoints.at<cv::Vec2d>(0,0);
    return cv::Point2d(p2[0],p2[1]);
}

cv::Point3d CameraParameters::ray(cv::Point2d pt) {

    cv::Mat pt_mat(1, 1, CV_64FC2, cv::Scalar(pt.x, pt.y));
    cv::Mat undistored;
    cv::Matx33d cameraMatrix(
            fx(), 0, cx(),
            0, fy(), cy(),
            0,  0,  1);
    cv::Vec<double, 5> distCoeffs(k1(), k2(), p1(), p2(), k3());
    cv::undistortPoints(pt_mat, undistored, cameraMatrix, distCoeffs);
    assert(undistored.type() == CV_64FC2);
    cv::Vec2d undistored_vec = undistored.at<cv::Vec2d>(0,0);
    cv::Matx31d undistored_vec3 (undistored_vec[0], undistored_vec[1], 1);

    cv::Matx31d p = R().t() * (undistored_vec3 - T());
    return cv::Point3d(p(0), p(1), p(2));
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