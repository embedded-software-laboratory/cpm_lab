#include "cpm_tools/default.h"
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "CameraParameters/CameraParameters.h"

int main(int argc, char* argv[]) {
    if(argc != 2) {
        cout << "Usage: calculate_camera_pose_calibration <path to intrinsic parameter YAML file>" << endl;
        return 1;
    }

    CameraParameters params;
    params.setIntrinsicParametersFromYAML(argv[1]);


    // Read 2D/3D point pairs
    double x,y,X,Y,Z;
    vector<cv::Point3d> objPts;
    vector<cv::Point2d> imgPts;
    while(scanf("(%lf,%lf),(%lf,%lf,%lf)\n", &x,&y,&X,&Y,&Z) != EOF) {
        imgPts.emplace_back(x,y);
        objPts.emplace_back(X,Y,Z);
    }

    // Calculate
    params.setExtrinsicParametersFromPnP(objPts, imgPts);

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