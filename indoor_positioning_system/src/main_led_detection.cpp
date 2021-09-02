#include <pylon/PylonIncludes.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <pylon/usb/BaslerUsbInstantCamera.h>
#pragma GCC diagnostic pop
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
#include "cpm/init.hpp"
#include "cpm/Writer.hpp"
#include "cpm/get_time_ns.hpp"


using namespace Pylon;
using namespace Basler_UsbCameraParams;
using namespace std;

typedef Pylon::CBaslerUsbInstantCamera Camera_t;
typedef Pylon::CBaslerUsbImageEventHandler ImageEventHandler_t; // Or use Camera_t::ImageEventHandler_t
typedef Pylon::CBaslerUsbGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t

/**
 * \struct FrameInfo
 * \brief Stores important information of a frame, i.e., timestamp, image itself, and detected points.
 * \ingroup ips
 */
struct FrameInfo
{
    //! Image created by the camera
    cv::Mat image;
    //! Timestamp at which the image was created by the camera
    uint64_t timestamp;
    //! x-coordinate of point detected in image
    std::vector<double> points_x;
    //! y-coordinate of point detected in image
    std::vector<double> points_y;
};

//! Saves, whether the user enabled visualization
bool enable_visualization;
//! Saves, whether the user enabled additional debugging output
bool enable_debug;

//! Used from \link worker_grab_image \endlink to provide the retreived images for \link worker_led_detection \endlink.
ThreadSafeQueue< std::shared_ptr<FrameInfo> > queue_frames;
//! If visualization is enabled, the detected points are given here to \link worker_visualization \endlink.
ThreadSafeQueue< std::shared_ptr<FrameInfo> > queue_visualization;

/**
 * \brief This method retrieves images provided by the thread executing \link worker_grab_image \endlink,
 * finds contours of LED points in these images, and outputs these points via the DDS topic "ipsLedPoints".
 * \ingroup ips
 */
void worker_led_detection()
{
    cpm::Writer<LedPoints> LED_writer("ipsLedPoints");

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
            // auto t =  std::to_string(cpm::get_time_ns());
            // cv::imwrite("debug_" + t + "_raw.png",frame->image);
            // cv::imwrite("debug_" + t + "_thresh.png",img_binary);
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

        
        if (enable_debug)
        {
            cv::hconcat(frame->image, img_binary, frame->image);
            
            // Draw contours
            cv::Mat img_contours = cv::Mat::zeros( img_binary.size(), CV_8UC1 );
            for( size_t i = 0; i < contours.size(); i++ )
            {
                double size = cv::contourArea(contours[i]);
                if (size < 60 && size > 3) {
                    cv::Scalar color = cv::Scalar( 255,255,255 );
                    cv::drawContours( img_contours, contours, i, color, 3);
                }
            }
            cv::hconcat(frame->image, img_contours, frame->image);
        }
        if(enable_visualization) queue_visualization.push(frame);
    }
}

/**
 * \brief If enabled this method visualizes the detected points until ESC is pressed.
 * \ingroup ips
 */
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

/**
 * \brief This method sets up all relevant parameters for the IPS camera and retreives frames until
 * the process gets terminated. The images are given via a queue to the thread executing
 * \link worker_led_detection \endlink.
 * \ingroup ips
 */
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
        camera.ExposureTime.SetValue(120);

        // Start grabbing by using the strategy OneByOne. This strategy is chosen to
        // avoid intentional frame drops because frame drops would lead to a tracking
        // reset due to the vehicle ID-detection via different LED-frequencies.
        // Additionally, the strategy LatestImageOnly is not needed here, since the
        // program is fast enough to handle "old" images in just a few milliseconds.
        camera.StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_OneByOne);
        GrabResultPtr_t ptrGrabResult;

        int frameCount = 0;
        uint64_t lastFrameReportTime = cpm::get_time_ns();
        uint64_t lastFrameTimeStamp = cpm::get_time_ns();
        uint64_t max_time_between_frames = 0;
        uint64_t min_time_between_frames = 1000000000;
        uint64_t max_retrieve_time = 0;


        // The camera counts time in nanoseconds, from an arbitrary starting point.
        // Record the camera and computer clock at the same time.
        // Use the difference to correct the timestamps.
        camera.TimestampLatch();
        const uint64_t startTime = cpm::get_time_ns();
        const int64_t startTicks = camera.TimestampLatchValue.GetValue();
        while(camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            // RetrieveResult calls the image event handler's OnImageGrabbed method.
            
            uint64_t time_before_retrieve = cpm::get_time_ns();
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
            max_retrieve_time = max(max_retrieve_time, cpm::get_time_ns() - time_before_retrieve);


            if(ptrGrabResult->GrabSucceeded())
            {
                ++frameCount;

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


                uint64_t now = cpm::get_time_ns();
                max_time_between_frames = max(max_time_between_frames, now - lastFrameTimeStamp);
                min_time_between_frames = min(min_time_between_frames, now - lastFrameTimeStamp);
                lastFrameTimeStamp = now;

                // report actual FPS
                if(frameCount % 100 == 0)
                {
                    uint64_t now = cpm::get_time_ns();
                    double fps = 100.0/((now - lastFrameReportTime)*1e-9);

                    if (fps < 49.5){
                        std::cout << now << " max time between frames: " << max_time_between_frames << " min time between frames: " << min_time_between_frames << " FPS " << fps << " comparably low!" << std::endl;
                    }
                    else {
                        std::cout << now << " max time between frames: " << max_time_between_frames << " min time between frames: " << min_time_between_frames << " FPS " << fps << std::endl;
                    }
                    cout << "Max. TTW: " << max_retrieve_time << endl;

                    max_time_between_frames = 0;
                    min_time_between_frames = 1000000000;
                    max_retrieve_time = 0;

                    lastFrameReportTime = now;
                }
            }
            else
            {
                cerr << "Error, image grab failed." << endl;
                cout << "Error, image grab failed." << endl;
            }

        }
    }
    catch (const GenericException &e)
    {
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
    }

    PylonTerminate(); 
}

/**
 * \brief This process retreives images from the camera, detects LEDs of the vehicles and sends
 * a list with these detected points via DDS to the process \link main_ips_pipeline.cpp \endlink. It starts all
 * relevant workers in separate threads. For debugging purposes the parameter --visualization=1
 * can be used to activate a visualization of the detected points and the parameter --debug can be used
 * to get additionally information.
 * \ingroup ips
 */
int main(int argc, char* argv[])
{
    if(argc < 2) {
        std::cout << "To enable visualization use parameter --visualization=1" << std::endl;
    }

    cpm::init(argc, argv);

    enable_visualization = cpm::cmd_parameter_bool("visualization", false, argc, argv);
    enable_debug = cpm::cmd_parameter_bool("debug", false, argc, argv);

    std::thread thread_led_detection([](){worker_led_detection();});
    std::thread thread_visualization;
    if(enable_visualization)
    {
        thread_visualization = std::thread([](){worker_visualization();});
    }
    worker_grab_image();
    return 0;
}