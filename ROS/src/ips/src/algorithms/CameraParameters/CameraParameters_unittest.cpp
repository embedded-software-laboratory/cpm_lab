#include "CameraParameters.h"
#include "tools/unittest/catch.hpp"

TEST_CASE("CameraParameters") {

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

    SECTION("forward reverse projection consistency") {
        const int cols = 1000;

        cv::Mat3d points(1, cols);

        // random 3D points
        uint32_t seed = 329845;
        for (int j = 0; j < cols; ++j) {
            seed *= 16807;
            double x = double(seed) / 4294967296.0 * 4;
            seed *= 16807;
            double y = double(seed) / 4294967296.0 * 4;
            seed *= 16807;
            double z = double(seed) / 4294967296.0;
            points.at<cv::Vec3d>(0, j) = cv::Vec3d(x, y, z);
        }

        // forward projection
        cv::Mat2d pixels = params.project(points);

        // reverse projection
        cv::Vec3d origin;
        cv::Mat3d directions;
        std::tie(origin, directions) = params.pixelRays(pixels);

        // test if each ray passes close to its corresponding point
        for (int j = 0; j < cols; ++j) {
            auto direction = directions.at<cv::Vec3d>(0, j);
            auto point = points.at<cv::Vec3d>(0, j);

            // find point on each ray that is closest to the original point
            double ray_param = (direction.dot(point - origin)) / (direction.dot(direction));
            auto point_reconstruction = origin + ray_param * direction;
            auto error_vector = point_reconstruction - point;
            for (auto err:error_vector.val) {
                CHECK(fabs(err) < 0.001);
            }
        }

    }

    SECTION("same ray same pixel") {

        // find the ray of some arbitrary pixel
        const double pixel_x = 123.777;
        const double pixel_y = 546.423;
        cv::Mat2d pixels(1, 1, cv::Vec2d(pixel_x,pixel_y));
        cv::Vec3d origin;
        cv::Mat3d directions;
        std::tie(origin, directions) = params.pixelRays(pixels);

        // generate multiple points on the same ray
        const int cols = 100;
        cv::Mat3d points(1, cols);
        for (int j = 0; j < cols; ++j) {
            points.at<cv::Vec3d>(0, j) = origin + 0.2718281828 * (j+1) * directions.at<cv::Vec3d>(0, 0);
        }

        // forward projection
        pixels = params.project(points);

        // test that all points on the ray fall on the same pixel
        for (int j = 0; j < cols; ++j) {
            cv::Vec2d pixel = pixels.at<cv::Vec2d>(0, j);
            CHECK( fabs(pixel[0] - pixel_x) < 0.01 );
            CHECK( fabs(pixel[1] - pixel_y) < 0.01 );
        }
    }

    SECTION("recover extrinsic parameters") {

        // second set of parameters, that initially only share the intrinsic parameters
        CameraParameters params2;
        params2.fx = params.fx;
        params2.fy = params.fy;
        params2.cx = params.cx;
        params2.cy = params.cy;
        params2.k1 = params.k1;
        params2.k2 = params.k2;
        params2.k3 = params.k3;
        params2.p1 = params.p1;
        params2.p2 = params.p2;

        const int cols = 10;

        cv::Mat3d points(1, cols);

        // random 3D points
        uint32_t seed = 329845;
        for (int j = 0; j < cols; ++j) {
            seed *= 16807;
            double x = double(seed) / 4294967296.0 * 4;
            seed *= 16807;
            double y = double(seed) / 4294967296.0 * 4;
            seed *= 16807;
            double z = double(seed) / 4294967296.0;
            points.at<cv::Vec3d>(0, j) = cv::Vec3d(x, y, z);
        }

        // forward projection
        cv::Mat2d pixels = params.project(points);

        // convert types, convert pixel coordinates to integers (because that is what the camera does)
        std::vector<cv::Point3d> objPts;
        std::vector<cv::Point2d> imgPts;
        for (int j = 0; j < cols; ++j) {
            objPts.push_back(cv::Point3d(points.at<cv::Vec3d>(0, j)));
            imgPts.push_back(cv::Point2i(pixels.at<cv::Vec2d>(0, j)));
        }

        // recover extrinsic parameters
        params2.setExtrinsicsFromPnP(objPts, imgPts);

        // compare extrinsic parameters
        for (int i = 0; i < 3; ++i) {
            CHECK( fabs(params.T()[i] - params2.T()[i]) < 0.01 );
            for (int j = 0; j < 3; ++j) {
                CHECK( fabs(params.R()(i,j) - params2.R()(i,j)) < 0.01 );
            }
        }
    }
}