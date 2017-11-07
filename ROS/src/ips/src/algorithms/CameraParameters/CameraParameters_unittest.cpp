#include "CameraParameters.h"
#include "tools/unittest/catch.hpp"


TEST_CASE("CameraParameters_forward_reverse_projection_consistency") {

    CameraParameters params;

    params.fx = 945.444618;
    params.fy = 898.791897;
    params.cx = 776.656112;
    params.cy = 533.951952;
    params.k1 = -0.253795;
    params.k2 = 0.041498;
    params.k3 = 0.0001;
    params.p1 = 0.019698;
    params.p2 = 0.001308;
    params.R = cv::Matx33d(0.890911892221145,  -0.371201178136418,   0.261697698975504,  -0.046580919372195,  -0.647844401480301,  -0.760347190052723,   0.451781161901843,   0.665212234419035,  -0.594463173737327);
    params.T = cv::Vec3d(-0.521833,0.446593,0.949944);


    const int rows = 1;
    const int cols = 1000;


    cv::Mat3d points(rows,cols);

    // random 3D points
    uint32_t seed = 329845;
    for (int j = 0; j < cols; ++j) {
        seed *= 16807;
        double x = double(seed) / 4294967296.0 * 4;
        seed *= 16807;
        double y = double(seed) / 4294967296.0 * 4;
        seed *= 16807;
        double z = double(seed) / 4294967296.0;
        points.at<cv::Vec3d>(0,j) = cv::Vec3d(x,y,z);
    }

    // forward projection
    cv::Mat2d pixels = params.project(points);

    // reverse projection
    cv::Vec3d origin;
    cv::Mat3d directions;
    std::tie(origin, directions) = params.pixelRays(pixels);

    // find point on each ray that is closest to the original point
    double max_error = 0;
    for (int j = 0; j < cols; ++j) {
        auto direction = directions.at<cv::Vec3d>(0,j);
        auto point = points.at<cv::Vec3d>(0,j);
        double ray_param = (direction.dot(point-origin))/(direction.dot(direction));
        auto point_reconstruction = origin + ray_param * direction;
        auto error_vector = point_reconstruction - point;
        for(auto err:error_vector.val) {
            max_error = fmax(max_error, fabs(err));
        }
    }
    CHECK( max_error < 0.001 );
}