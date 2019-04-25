#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <thread>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Pylon;
using namespace Basler_UsbCameraParams;
using namespace std;

typedef Pylon::CBaslerUsbInstantCamera Camera_t;
typedef Pylon::CBaslerUsbImageEventHandler ImageEventHandler_t; // Or use Camera_t::ImageEventHandler_t
typedef Pylon::CBaslerUsbGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t


std::mutex visualization_mutex;
cv::Mat visualization_image;




uint64_t get_time_ns()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Only look for cameras supported by Camera_t
        CDeviceInfo info;
        info.SetDeviceClass( Camera_t::DeviceClass());

        // Create an instant camera object with the first found camera device that matches the specified device class.
        Camera_t camera( CTlFactory::GetInstance().CreateFirstDevice( info));

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // Open the camera.
        camera.Open();

        // A GenICam node map is required for accessing chunk data. That's why a small node map is required for each grab result.
        // Creating a lot of node maps can be time consuming.
        // The node maps are usually created dynamically when StartGrabbing() is called.
        // To avoid a delay caused by node map creation in StartGrabbing() you have the option to create
        // a static pool of node maps once before grabbing.
        camera.StaticChunkNodeMapPoolSize = camera.MaxNumBuffer.GetValue();

        // Enable chunks in general.
        if (GenApi::IsWritable(camera.ChunkModeActive))
        {
            camera.ChunkModeActive.SetValue(true);
        }
        else
        {
            throw RUNTIME_EXCEPTION( "The camera doesn't support chunk features");
        }

        // Enable time stamp chunks.
        camera.ChunkSelector.SetValue(ChunkSelector_Timestamp);
        camera.ChunkEnable.SetValue(true);

        // Enable CRC checksum chunks.
        camera.ChunkSelector.SetValue(ChunkSelector_PayloadCRC16);
        camera.ChunkEnable.SetValue(true);

        // Set fixed FPS
        camera.AcquisitionFrameRateEnable.SetValue(true);
        camera.AcquisitionFrameRate.SetValue(50);
        camera.DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitMode_Off);


        // Set lowest gain, results in lowest image noise
        camera.GainAuto.SetValue(GainAuto_Off);
        camera.Gain.SetValue(camera.Gain.GetMin());

        // Set exposure
        camera.ExposureAuto.SetValue(ExposureAuto_Off);
        camera.ExposureTime.SetValue(80);


        camera.StartGrabbing();
        GrabResultPtr_t ptrGrabResult;

        int frameCount = 0;
        uint64_t lastFrameReportTime = get_time_ns();

        while(camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            // RetrieveResult calls the image event handler's OnImageGrabbed method.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);


            if(ptrGrabResult->GrabSucceeded())
            {
                frameCount++;

                // The result data is automatically filled with received chunk data.
                // (Note:  This is not the case when using the low-level API)
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                //cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl;

                // Check to see if a buffer containing chunk data has been received.
                if (PayloadType_ChunkData != ptrGrabResult->GetPayloadType())
                {
                    throw RUNTIME_EXCEPTION( "Unexpected payload type received.");
                }

                // Since we have activated the CRC Checksum feature, we can check
                // the integrity of the buffer first.
                // Note: Enabling the CRC Checksum feature is not a prerequisite for using
                // chunks. Chunks can also be handled when the CRC Checksum feature is deactivated.
                if (ptrGrabResult->HasCRC() && ptrGrabResult->CheckCRC() == false)
                {
                    throw RUNTIME_EXCEPTION( "Image was damaged!");
                }


                //if (IsReadable(ptrGrabResult->ChunkTimestamp))
                //    cout << "TimeStamp (Result): " << ptrGrabResult->ChunkTimestamp.GetValue() << endl;


                cv::Mat img = (cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, pImageBuffer)).clone();

                

                cv::Mat img_binary;
                cv::threshold(img, img_binary, 127, 255, cv::THRESH_BINARY);

                std::vector<double> points_x;
                std::vector<double> points_y;
                std::vector<std::vector<cv::Point> > contours;

                cv::findContours(img_binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

                for (std::vector<cv::Point> contour : contours) {
                    double size = cv::contourArea(contour);
                    if (size < 15 && size > 6) {
                        cv::Moments M = cv::moments(contour);
                        points_x.push_back((M.m10 / (M.m00 + 1e-5)));
                        points_y.push_back((M.m01 / (M.m00 + 1e-5)));
                    }
                }


                // Visualization
                
                {
                    cv::Mat img_small;
                    cv::resize(img, img_small, cv::Size(), 0.5, 0.5);
                    cv::Mat img_small_BGR;
                    cv::cvtColor(img_small, img_small_BGR, cv::COLOR_GRAY2BGR);

                    for (size_t i = 0; i < points_x.size(); ++i)
                    {
                        cv::circle(img_small_BGR, cv::Point(points_x[i]/2, points_y[i]/2), 7, cv::Scalar(0,0,255));
                    }

                    cv::imshow( "", img_small_BGR ); 
                    cv::waitKey(1);
                }

                // Update visualization Data
                /*{
                    std::lock_guard<std::mutex> lock(visualization_mutex);
                    img.copyTo(visualization_image);
                }*/


                if(frameCount % 100 == 0)
                {
                    uint64_t now = get_time_ns();
                    double fps = 100.0/((now - lastFrameReportTime)*1e-9);
                    std::cout << "FPS " << fps << std::endl;
                    lastFrameReportTime = now;
                }
            }
            else
            {
                cerr << "Error, image grab failed." << endl;
            }

        }
    }
    catch (const GenericException &e)
    {
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources. 
    PylonTerminate(); 

    return exitCode;
}

