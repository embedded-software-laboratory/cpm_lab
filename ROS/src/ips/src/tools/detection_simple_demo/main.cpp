
#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include "DetectionDispatcherLogic/DetectionDispatcherLogic.h"
#include "AprilTagDetector/AprilTagDetector.h"
#include "utils/ThreadSafeQueue.h"

using namespace Pylon;
using namespace GenApi;

bool loop = true;

bool grabImage(shared_ptr<CInstantCamera> camera, WithTimestamp<cv::Mat> &image_copy) {

    CGrabResultPtr ptrGrabResult;
    if (!camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_Return)) {
        cout << "RetrieveResult() failed" << endl;
        return false;
    }
    auto timestamp = ros::Time::now();

    int rows = ptrGrabResult->GetHeight();
    int cols = ptrGrabResult->GetWidth();
    void *data = ptrGrabResult->GetBuffer();
    size_t stride;
    assert(ptrGrabResult->GetStride(stride));
    assert(ptrGrabResult->GetPixelType() == PixelType_Mono8);
    cv::Mat image_tmp(rows, cols, CV_8UC1, data, stride);
    image_tmp.copyTo(image_copy);
    image_copy.timestamp = timestamp;
    return true;
}



template<size_t input_queue_size, size_t output_queue_size>
void detectLoop(
    ThreadSafeQueue< tuple<ros::Time, cv::Mat1b, cv::Point>,  input_queue_size > &input_queue,
    ThreadSafeQueue< vector<AprilTagDetectionStamped>, output_queue_size  > &output_queue
) {

    tuple<ros::Time, cv::Mat1b, cv::Point> input_data;

    AprilTagDetector aprilTagDetector(AprilTagFamily::Tag36h11);

    while(loop && input_queue.read(input_data)) {
        ros::Time timestamp;
        cv::Mat1b img;
        cv::Point offset;
        tie(timestamp, img, offset) = input_data;
        auto detections = aprilTagDetector.detect(img, offset);
        vector<AprilTagDetectionStamped> detections_stamped;
        for(auto const & detection:detections) detections_stamped.emplace_back(detection, timestamp);
        output_queue.write(detections_stamped);
    }
}



void grabLoop(shared_ptr<CInstantCamera> camera) {

    ThreadSafeQueue< tuple<ros::Time, cv::Mat1b, cv::Point>, 100  > detect_crop_input_queue;
    ThreadSafeQueue< std::vector<AprilTagDetectionStamped>, 100  > detect_crop_result_queue;

    ThreadSafeQueue< tuple<ros::Time, cv::Mat1b, cv::Point>, 1  > detect_full_input_queue;
    ThreadSafeQueue< std::vector<AprilTagDetectionStamped>, 1  > detect_full_result_queue;


    vector<std::thread> detection_threads;
    for (int i = 0; i < 5; ++i) {
        detection_threads.emplace_back([&](){detectLoop(detect_crop_input_queue, detect_crop_result_queue);});
    }
    detection_threads.emplace_back([&](){detectLoop(detect_full_input_queue, detect_full_result_queue);});



    CameraParameters params;
    params.fx = 1;
    params.fy = 1;
    params.cx = 0;
    params.cy = 0;
    params.k1 = 0;
    params.k2 = 0;
    params.k3 = 0;
    params.p1 = 0;
    params.p2 = 0;
    params.R = cv::Matx33d(1,0,0,0,1,0,0,0,1);
    params.T = cv::Vec3d(0,0,0);
    DetectionDispatcherLogic detectionDispatcherLogic(params);

    vector<cpm_msgs::VehicleState> vehicle_states;
    {
        cpm_msgs::VehicleState v;
        v.id = 1;
        v.pose.position.x = NaN;
        vehicle_states.push_back(v);
    }
    {
        cpm_msgs::VehicleState v;
        v.id = 51;
        v.pose.position.x = NaN;
        vehicle_states.push_back(v);
    }
    {
        cpm_msgs::VehicleState v;
        v.id = 72;
        v.pose.position.x = NaN;
        vehicle_states.push_back(v);
    }


    const string serial_no(camera->GetDeviceInfo().GetSerialNumber().c_str());


    int active_crop_detection_jobs = 0;
    cv::Mat previous_image;
    while(loop)
    {

        // Get new image
        WithTimestamp<cv::Mat> image;
        if(!grabImage(camera, image)) {
            loop = false;
            break;
        }


        vector<AprilTagDetectionStamped> detections;

        // Get previous detections
        while (active_crop_detection_jobs > 0) {
            vector<AprilTagDetectionStamped> detection;
            assert(detect_crop_result_queue.read(detection));
            detections.insert( detections.end(), detection.begin(), detection.end() );
            active_crop_detection_jobs--;
        }

        // Try to get full frame detections
        {
            vector<AprilTagDetectionStamped> detection;
            if(detect_full_result_queue.read_nonblocking(detection)){
                detections.insert( detections.end(), detection.begin(), detection.end() );
            }
        }




        // determine detection jobs for new image
        vector<cv::Rect> ROIs;
        bool full_frame_detection;
        vector<optional<AprilTagDetectionStamped>> detection_per_vehicle;
        tie(ROIs, full_frame_detection, detection_per_vehicle) = detectionDispatcherLogic.apply( detections, vehicle_states );

        cv::Rect full_frame_ROI(0,0,image.cols,image.rows);

        // Start detection job for each crop
        for(auto ROI:ROIs) {
            ROI &= full_frame_ROI;
            if(ROI.area() > 0) {
                cv::Mat crop = image(ROI);
                assert(detect_crop_input_queue.write(make_tuple(image.timestamp, crop, ROI.tl())));
                active_crop_detection_jobs++;
            }
        }

        // Try to start full frame detection
        if(full_frame_detection) {
            detect_full_input_queue.write_nonblocking(make_tuple(image.timestamp, image, cv::Point(0,0)));
        }


        // visualize
        if(previous_image.rows > 0 && previous_image.cols > 0) {
            cv::cvtColor(previous_image, previous_image, CV_GRAY2BGR);

            for (auto const &detection:detection_per_vehicle) {
                if(detection)
                {
                    for (int k = 0; k < 4; ++k) {
                        int m = (k + 1) % 4;
                        cv::line(previous_image,
                                 cv::Point(detection.value().points[k][0], detection.value().points[k][1]),
                                 cv::Point(detection.value().points[m][0], detection.value().points[m][1]),
                                 k ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 5);
                    }
                }
            }

            cv::resize(previous_image, previous_image, cv::Size(), 0.5, 0.5);

            cv::imshow(serial_no, previous_image);

            int key = cv::waitKey(1);
            if (key == 27) loop = false;
        }

        image.copyTo(previous_image);
    }


    // cleanup
    detect_crop_input_queue.close();
    detect_crop_result_queue.close();
    detect_full_input_queue.close();
    detect_full_result_queue.close();

    for(auto &t:detection_threads) t.join();
}


int main(int argc, char* argv[])
{
    int exitCode = 0;
    PylonInitialize();

    try
    {

        vector<shared_ptr<CInstantCamera>> cameras;
        //cameras.push_back(make_shared<CInstantCamera>(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21704342"))));
        cameras.push_back(make_shared<CInstantCamera>(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21967260"))));


        for ( size_t i = 0; i < cameras.size(); ++i)
        {
            cout << "Using device " << cameras[ i ]->GetDeviceInfo().GetModelName()  << "   SN  "  << cameras[i]->GetDeviceInfo().GetSerialNumber() << endl;

            cameras[ i ]->RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
            cameras[ i ]->StartGrabbing();

            INodeMap& nodemap = cameras[ i ]->GetNodeMap();

            CEnumerationPtr gainAuto( nodemap.GetNode( "GainAuto"));
            gainAuto->FromString("Off");

            CEnumerationPtr expauto( nodemap.GetNode( "ExposureAuto"));
            expauto->FromString("Off");

            CFloatPtr gain( nodemap.GetNode( "Gain"));
            gain->SetValue(5);

            CFloatPtr exp( nodemap.GetNode( "ExposureTime"));
            exp->SetValue(1000);
        }


        ros::Time::init();


        vector<thread> grabThreads;
        for ( size_t i = 0; i < cameras.size(); ++i) {
            auto sp_camera = cameras[i];
            grabThreads.emplace_back([=](){grabLoop(sp_camera);});
        }

        // trigger Loop
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while(loop) {
            for (size_t i = 0; i < cameras.size(); ++i) {
                cameras[i]->ExecuteSoftwareTrigger();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }

        for(auto &t:grabThreads) t.join();
        cameras.clear();
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