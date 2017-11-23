#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "CameraParameters/CameraParameters.h"

int main(int argc, char* argv[]) {
    if(argc != 2) {
        cout << "Usage: calculate_camera_pose_calibration <path to intrinsic parameter yaml file>" << endl;
        return 1;
    }

    // Read camera parameters
    YAML::Node intrinsic_params = YAML::LoadFile(argv[1]);

    CameraParameters params;
    params.fx = intrinsic_params["fx"].as<double>();
    params.fy = intrinsic_params["fy"].as<double>();
    params.cx = intrinsic_params["cx"].as<double>();
    params.cy = intrinsic_params["cy"].as<double>();
    params.k1 = intrinsic_params["k1"].as<double>();
    params.k2 = intrinsic_params["k2"].as<double>();
    params.k3 = intrinsic_params["k3"].as<double>();
    params.p1 = intrinsic_params["p1"].as<double>();
    params.p2 = intrinsic_params["p2"].as<double>();


    // Read 2D/3D point pairs
    double x,y,X,Y,Z;
    vector<cv::Point3d> objPts;
    vector<cv::Point2d> imgPts;
    while(scanf("(%lf,%lf),(%lf,%lf,%lf)\n", &x,&y,&X,&Y,&Z) != EOF) {
        imgPts.emplace_back(x,y);
        objPts.emplace_back(X,Y,Z);
    }

    // Calculate
    params.setExtrinsicsFromPnP(objPts, imgPts);

    // Output parameters
    cout << "R: [";
    for (int i = 0; i < 9; ++i) {
        if(i) cout << ", ";
        cout << params.R().val[i];
    }
    cout << "]" << endl << "T: [";
    for (int i = 0; i < 3; ++i) {
        if(i) cout << ", ";
        cout << params.T().val[i];
    }
    cout << "]" << endl;
}