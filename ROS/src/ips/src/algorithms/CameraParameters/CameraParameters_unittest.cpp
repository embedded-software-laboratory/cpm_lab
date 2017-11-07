#include "CameraParameters.h"
#include "tools/unittest/catch.hpp"

#include <iostream>
using namespace std;

TEST_CASE("CameraParameters") {

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
    params.R = cv::Matx33d(   0.890911892221145,  -0.371201178136418,   0.261697698975504,  -0.046580919372195,  -0.647844401480301,  -0.760347190052723,   0.451781161901843,   0.665212234419035,  -0.594463173737327);
    params.T = cv::Vec3d(-0.521833,0.446593,0.949944);

    auto p1 = params.ray(cv::Point2d(100,200));
    for (int i = 0; i < 100; ++i)
    {
        p1 = params.ray(params.project(p1));
        cout << p1 << endl;

        /*
        auto p2 = p1;
        double l = sqrt(p2.x*p2.x + p2.y*p2.y + p2.z*p2.z);
        p2.x /= l;
        p2.y /= l;
        p2.z /= l;
        cout << p2 << endl;
        */
    }
}