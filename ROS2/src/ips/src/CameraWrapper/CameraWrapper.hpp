#pragma once
#include "cpm_tools/default.hpp"
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include "cpm_tools/CpmNode.hpp"


template <typename Base>
struct WithTimestamp : public Base {
    uint64_t timestamp;
    WithTimestamp(){}
    WithTimestamp(const Base &base, uint64_t timestamp):Base(base),timestamp(timestamp){}
    WithTimestamp(Base &&base, uint64_t timestamp):Base(std::move(base)),timestamp(timestamp){}
};


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