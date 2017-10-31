#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <unistd.h>
#include <pylon/PylonIncludes.h>

using namespace Pylon;
using namespace GenApi;
using namespace std;

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

    try
    {
        CInstantCameraArray cameras( 2 );
        cameras[0].Attach(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21704342")));
        cameras[1].Attach(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21967260")));

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
            gain->SetValue(18);

            CFloatPtr exp( nodemap.GetNode( "ExposureTime"));
            exp->SetValue(50);
        }

        for (int j = 0; j < 30; ++j)
        {
            usleep(1000000/50);

            /*for ( size_t i = 0; i < cameras.GetSize(); ++i) {
                cameras[ i ].ExecuteSoftwareTrigger();
            }*/

            
            cameras[ 0 ].ExecuteSoftwareTrigger();
            usleep(1050);
            cameras[ 1 ].ExecuteSoftwareTrigger();

            usleep(500);
            
            for ( size_t i = 0; i < cameras.GetSize(); ++i)
            {
                CGrabResultPtr ptrGrabResult;
                if (!cameras[ i ].RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException)) {
                    cout << "RetrieveResult failed" << endl;
                    return 1;
                }

                intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();

                auto SN = cameras[ cameraContextValue ].GetDeviceInfo().GetSerialNumber();
                std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
                std::time_t time = std::chrono::system_clock::to_time_t(tp);
                std::tm timetm = *std::localtime(&time);

                auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()).count() % 1000000;
                std::ostringstream ss;
                ss << std::put_time(&timetm, "%Y-%m-%d_%H-%M-%S") << "."
                << std::setw( 6 ) << std::setfill( '0' ) << microsec << "--" << j << "--" << SN;


                CImagePersistence::Save(ImageFileFormat_Tiff, ("img/" + ss.str()  + ".tif").c_str(), ptrGrabResult);

            }
        } 
    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    PylonTerminate(); 

    return exitCode;
}