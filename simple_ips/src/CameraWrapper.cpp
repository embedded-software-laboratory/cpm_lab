#include "CameraWrapper.hpp"


CameraWrapper::CameraWrapper(string serial_number):serial_number(serial_number) {
    try
    {
        auto &pylon_factory = Pylon::CTlFactory::GetInstance();
        auto device_info = Pylon::CDeviceInfo().SetSerialNumber(serial_number.c_str());
        auto device = pylon_factory.CreateDevice(device_info);
        camera = make_shared<Pylon::CInstantCamera>(device);

        camera->RegisterConfiguration(new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll,
                                      Pylon::Cleanup_Delete);
        camera->StartGrabbing();
    }
    catch (const GenericException &e)
    {
        throw std::runtime_error("Cannot open camera with serial number " + serial_number + ": " + string(e.GetDescription()));
    }
}

void CameraWrapper::setGainExposure(double gain_val, double exposure_val) {
    GenApi::INodeMap& nodemap = camera->GetNodeMap();

    GenApi::CEnumerationPtr gainAuto( nodemap.GetNode( "GainAuto"));
    gainAuto->FromString("Off");

    GenApi::CEnumerationPtr expauto( nodemap.GetNode( "ExposureAuto"));
    expauto->FromString("Off");

    GenApi::CFloatPtr gain( nodemap.GetNode( "Gain"));
    gain->SetValue(gain_val);

    GenApi::CFloatPtr exp( nodemap.GetNode( "ExposureTime"));
    exp->SetValue(exposure_val);
}

void CameraWrapper::triggerExposure() {
    camera->ExecuteSoftwareTrigger();
}

void CameraWrapper::close() {
    camera->StopGrabbing();
    camera->Close();
}

string CameraWrapper::getSerialNumber() { return serial_number; }

bool CameraWrapper::grabImage(cv::Mat &image) {
    Pylon::CGrabResultPtr ptrGrabResult;
    try {
        if (!camera->RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_Return)) {
            if(camera->IsOpen()) cout << "RetrieveResult() failed" << endl;
            return false;
        }
    }
    catch (const GenericException &e) {
        cout << "Error in grabImage(): " << e.GetDescription() << endl;
        return false;
    }

    int rows = ptrGrabResult->GetHeight();
    int cols = ptrGrabResult->GetWidth();
    void *data = ptrGrabResult->GetBuffer();
    size_t stride;
    assert(ptrGrabResult->GetStride(stride));
    assert(ptrGrabResult->GetPixelType() == Pylon::PixelType_Mono8);
    cv::Mat image_tmp(rows, cols, CV_8UC1, data, stride);
    image_tmp.copyTo(image);
    return true;
}