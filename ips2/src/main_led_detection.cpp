#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <thread>
#include <mutex>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ThreadSafeQueue.hpp>
#include "LedPoints.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/CommandLineReader.hpp"
#include <dds/pub/ddspub.hpp>


using namespace Pylon;
using namespace Basler_UsbCameraParams;
using namespace std;

typedef Pylon::CBaslerUsbInstantCamera Camera_t;
typedef Pylon::CBaslerUsbImageEventHandler ImageEventHandler_t; // Or use Camera_t::ImageEventHandler_t
typedef Pylon::CBaslerUsbGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t

struct FrameInfo
{
    cv::Mat image;
    uint64_t timestamp;
    std::vector<double> points_x;
    std::vector<double> points_y;
};

bool enable_visualization;


ThreadSafeQueue< std::shared_ptr<FrameInfo> > queue_frames;
ThreadSafeQueue< std::shared_ptr<FrameInfo> > queue_visualization;

uint64_t get_time_ns()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

void worker_led_detection()
{

    auto LED_topic = cpm::get_topic<LedPoints>("ipsLedPoints");
    dds::pub::DataWriter<LedPoints> LED_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), LED_topic);

    // Number of points in the previous frame.
    // Used for debug trigger.
    size_t n_points_previous = 0;

    while (1)
    {
        std::shared_ptr<FrameInfo> frame;
        queue_frames.pop(frame);


        cv::Mat img_binary;
        cv::threshold(frame->image, img_binary, 127, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point> > contours;

        cv::findContours(img_binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        for (std::vector<cv::Point> contour : contours) {
            double size = cv::contourArea(contour);
            if (size < 60 && size > 3) {
                cv::Moments M = cv::moments(contour);
                frame->points_x.push_back((M.m10 / (M.m00 + 1e-5)));
                frame->points_y.push_back((M.m01 / (M.m00 + 1e-5)));
            }
        }

        // Debug output
        // Save the image for analysis, when the last vehicle disappears.
        if( frame->points_x.size() < 3 && n_points_previous >= 3 )
        {
            std::cout << "contours " << contours.size() << "   points " << frame->points_x.size() << std::endl;
            auto t =  std::to_string(get_time_ns());
            cv::imwrite("debug_" + t + "_raw.png",frame->image);
            cv::imwrite("debug_" + t + "_thresh.png",img_binary);
        }
        n_points_previous = frame->points_x.size();
        
        

        // publish the points to DDS
        LedPoints myledPoints;
        myledPoints.time_stamp().nanoseconds(frame->timestamp);
        myledPoints.led_points().resize(frame->points_x.size());
        for (size_t i = 0; i < frame->points_x.size(); ++i)
        {
            myledPoints.led_points().at(i) = ImagePoint(frame->points_x[i], frame->points_y[i]);
        }
        LED_writer.write(myledPoints);

        if(enable_visualization) queue_visualization.push(frame);
    }
}


void worker_visualization()
{
    while (1)
    {
        std::shared_ptr<FrameInfo> frame;
        queue_visualization.pop(frame);

        cv::Mat img_small;
        cv::resize(frame->image, img_small, cv::Size(), 0.5, 0.5);
        cv::Mat img_small_BGR;
        cv::cvtColor(img_small, img_small_BGR, cv::COLOR_GRAY2BGR);

        for (size_t i = 0; i < frame->points_x.size(); ++i)
        {
            cv::circle(img_small_BGR, cv::Point(frame->points_x[i]/2, frame->points_y[i]/2), 7, cv::Scalar(0,0,255));
        }

        cv::imshow( "BaslerLedDetection", img_small_BGR ); 
        if(cv::waitKey(1) == 27) // close on escape key
        {
            exit(0);
        }
    }
}

void worker_grab_image()
{
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
        camera.ExposureTime.SetValue(50);


        camera.StartGrabbing();
        GrabResultPtr_t ptrGrabResult;

        int frameCount = 0;
        uint64_t lastFrameReportTime = get_time_ns();


        // The camera counts time in nanoseconds, from an arbitrary starting point.
        // Record the camera and computer clock at the same time.
        // Use the difference to correct the timestamps.
        camera.TimestampLatch();
        const uint64_t startTime = get_time_ns();
        const int64_t startTicks = camera.TimestampLatchValue.GetValue();

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

                assert(IsReadable(ptrGrabResult->ChunkTimestamp));

                auto frame = std::make_shared<FrameInfo>();
                frame->timestamp = (ptrGrabResult->ChunkTimestamp.GetValue() - startTicks) + startTime;
                frame->image = (cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, pImageBuffer)).clone();
                queue_frames.push(frame);

                // report actual FPS
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
    }

    PylonTerminate(); 
}


int main(int argc, char* argv[])
{
    if(argc < 2) {
        std::cout << "To enable visualization use parameter --visualization=1" << std::endl;
    }
    enable_visualization = cpm::cmd_parameter_bool("visualization", false, argc, argv);

    std::thread thread_led_detection([](){worker_led_detection();});
    std::thread thread_visualization;
    if(enable_visualization)
    {
        thread_visualization = std::thread([](){worker_visualization();});
    }
    worker_grab_image();
    return 0;
}