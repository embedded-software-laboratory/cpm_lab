#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include "CameraParameters/CameraParameters.h"


cv::Point reproject(const CameraParameters& params, double x, double y, double z) {
    cv::Mat2d out = params.project(cv::Mat3d(1,1,cv::Vec3d(x,y,z)));
    cv::Vec2d out_pt = out.at<cv::Vec2d>(0,0);
    return cv::Point(out_pt[0], out_pt[1]);
}

int main(int argc, char* argv[]) {
    if(argc != 4) {
        cout << "Usage: validate_camera_calibration <path to calibration image> <path to intrinsic parameter YAML file> <path to extrinsic parameter YAML file>" << endl;
        return 1;
    }

    CameraParameters params;
    params.setIntrinsicParametersFromYAML(argv[2]);
    params.setExtrinsicParametersFromYAML(argv[3]);

    cv::Mat img = cv::imread(argv[1]);

    // Draw coordinate system
    for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 3; ++y) {
            cv::line(img, reproject(params,x,y + .2,0), reproject(params,x,y + .8,0), cv::Scalar(0,255,0),4);
            cv::line(img, reproject(params,y + .2,x,0), reproject(params,y + .8,x,0), cv::Scalar(0,0,255),4);
        }
    }

    for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 4; ++y) {
            cv::line(img, reproject(params,x,y,0), reproject(params,x,y,0.4), cv::Scalar(255,0,0),4);
        }
    }

    cv::imshow("img", img);
    while(cv::waitKey(0) != 27);


}