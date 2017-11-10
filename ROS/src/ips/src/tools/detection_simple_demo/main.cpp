
#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>


using namespace Pylon;
using namespace GenApi;


class CMyImageEventPrinter : public CImageEventHandler
{
public:

    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages)
    {
        std::cout << "OnImagesSkipped event for device " << camera.GetDeviceInfo().GetModelName();
        std::cout << countOfSkippedImages  << " images have been skipped." << std::endl;
        std::cout << std::endl;
    }


    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
        std::cout << "OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            std::cout << "SizeX: " << ptrGrabResult->GetWidth() << " -- ";
            std::cout << "SizeY: " << ptrGrabResult->GetHeight() << " -- ";
            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            std::cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << " ------ Timestamp " << ptrGrabResult->GetTimeStamp() << std::endl;
            std::cout << std::endl;
        }
        else
        {
            std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
        }
    }
};


int main(int argc, char* argv[])
{
    int exitCode = 0;
    PylonInitialize();
    cv::namedWindow("img");

    try
    {
        CInstantCameraArray cameras( 1 );
        //cameras[0].Attach(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21704342")));
        cameras[0].Attach(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21967260")));

        for ( size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cout << "Using device " << cameras[ i ].GetDeviceInfo().GetModelName()  << "   SN  "  << cameras[i].GetDeviceInfo().GetSerialNumber() << endl;

            cameras[ i ].RegisterImageEventHandler( new CMyImageEventPrinter, RegistrationMode_Append, Cleanup_Delete);
            cameras[ i ].RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
            cameras[ i ].StartGrabbing();

            INodeMap& nodemap = cameras[ i ].GetNodeMap();

            CEnumerationPtr gainAuto( nodemap.GetNode( "GainAuto"));
            gainAuto->FromString("Off");

            CEnumerationPtr expauto( nodemap.GetNode( "ExposureAuto"));
            expauto->FromString("Off");

            CFloatPtr gain( nodemap.GetNode( "Gain"));
            gain->SetValue(5);

            CFloatPtr exp( nodemap.GetNode( "ExposureTime"));
            exp->SetValue(10000);
        }

        bool loop = true;
        while(loop)
        {

            for ( size_t i = 0; i < cameras.GetSize(); ++i) {
                cameras[ i ].ExecuteSoftwareTrigger();
            }

            for ( size_t i = 0; i < cameras.GetSize(); ++i)
            {
                cv::Mat image_copy;

                {
                    CGrabResultPtr ptrGrabResult;
                    if (!cameras[i].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException)) {
                        cout << "RetrieveResult failed" << endl;
                        return 1;
                    }

                    intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();

                    auto SN = cameras[cameraContextValue].GetDeviceInfo().GetSerialNumber();

                    int rows = ptrGrabResult->GetHeight();
                    int cols = ptrGrabResult->GetWidth();
                    void *data = ptrGrabResult->GetBuffer();
                    size_t stride;
                    assert(ptrGrabResult->GetStride(stride));
                    assert(ptrGrabResult->GetPixelType() == PixelType_Mono8);
                    cv::Mat image_tmp(rows, cols, CV_8UC1, data, stride);
                    image_tmp.copyTo(image_copy);
                }


                resize(image_copy, image_copy, cv::Size(), 0.5, 0.5);

                cv::imshow("img", image_copy);

                int key = cv::waitKey(30);
                if(key == 27) loop = false;
            }
        }
    }
    catch (const GenericException &e)
    {
        // Error handling
        cout << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }

    PylonTerminate();

    return exitCode;
}