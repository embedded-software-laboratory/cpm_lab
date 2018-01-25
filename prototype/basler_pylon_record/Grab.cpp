// Grab.cpp
/*
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    This sample illustrates how to grab and process images using the CInstantCamera class.
    The images are grabbed and processed asynchronously, i.e.,
    while the application is processing a buffer, the acquisition of the next buffer is done
    in parallel.

    The CInstantCamera class uses a pool of buffers to retrieve image data
    from the camera device. Once a buffer is filled and ready,
    the buffer can be retrieved from the camera object for processing. The buffer
    and additional image data are collected in a grab result. The grab result is
    held by a smart pointer after retrieval. The buffer is automatically reused
    when explicitly released or when the smart pointer object is destroyed.
*/

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/ImagePersistence.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <pylon/usb/BaslerUsbInstantCamera.h>
typedef Pylon::CBaslerUsbInstantCamera Camera_t;
using namespace Basler_UsbCameraParams;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        CDeviceInfo info;
        info.SetDeviceClass( Camera_t::DeviceClass());
        Camera_t camera( CTlFactory::GetInstance().CreateFirstDevice( info));

        camera.Open();


        camera.GainAuto.SetValue(GainAuto_Off);
        camera.Gain.SetValue(0);
        camera.ExposureAuto.SetValue(ExposureAuto_Off);
        camera.ExposureTime.SetValue(10000);

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing();

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when c_countOfImagesToGrab images have been retrieved.
        int i = 4;
        while (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

                /*time_t now;
                time(&now);
                char buf[100];
                strftime(buf, 99, "%Y-%m-%d_%H-%M-%S", gmtime(&now));
                CImagePersistence::Save(ImageFileFormat_Tiff, ("img/" + std::string(buf) + ".tif").c_str(), ptrGrabResult);*/


                std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
                std::time_t time = std::chrono::system_clock::to_time_t(tp);
                std::tm timetm = *std::localtime(&time);

                auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()).count() % 1000000;
                std::ostringstream ss;
                ss << std::put_time(&timetm, "%Y-%m-%d_%H-%M-%S") << "."
                          << std::setw( 6 ) << std::setfill( '0' ) << microsec;

                //cout << ss.str();

                if((i++)%30==0)CImagePersistence::Save(ImageFileFormat_Png, ("img/" + ss.str() + ".png").c_str(), ptrGrabResult);
                

            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exitCode = 1;
    }

    PylonTerminate();  

    return exitCode;
}
