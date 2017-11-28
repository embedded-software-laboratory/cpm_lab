#pragma once
#include "utils/default.h"
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>

//! Wrapper for a Basler mono-camera. Exposes only necessary functionality
class CameraWrapper {
    Pylon::PylonAutoInitTerm init_handle;
    shared_ptr<Pylon::CInstantCamera> camera;
    string serial_number;

public:
    CameraWrapper(string serial_number);
    void setGainExposure(double gain_val, double exposure_val);
    void triggerExposure();
    string getSerialNumber();
    bool grabImage(WithTimestamp<cv::Mat> &image);
    void close();
};