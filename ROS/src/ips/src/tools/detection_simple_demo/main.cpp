// TODO refactor this mess


#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include "DetectionDispatcherLogic/DetectionDispatcherLogic.h"
#include "AprilTagDetector/AprilTagDetector.h"
#include "utils/ThreadSafeQueue.h"
#include <ros/package.h>
#include "CameraWrapper/CameraWrapper.h"

bool loop = true;

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



void grabLoop(CameraWrapper &camera) {

    ThreadSafeQueue< tuple<ros::Time, cv::Mat1b, cv::Point>, 100  > detect_crop_input_queue;
    ThreadSafeQueue< std::vector<AprilTagDetectionStamped>, 100  > detect_crop_result_queue;

    ThreadSafeQueue< tuple<ros::Time, cv::Mat1b, cv::Point>, 1  > detect_full_input_queue;
    ThreadSafeQueue< std::vector<AprilTagDetectionStamped>, 1  > detect_full_result_queue;


    vector<std::thread> detection_threads;
    for (int i = 0; i < 5; ++i) {
        detection_threads.emplace_back([&](){detectLoop(detect_crop_input_queue, detect_crop_result_queue);});
    }
    detection_threads.emplace_back([&](){detectLoop(detect_full_input_queue, detect_full_result_queue);});

    string extrinsic_parameters_path = ros::package::getPath("ips") + "/cfg/cameras/" + camera.getSerialNumber() + "/extrinsic_parameters.yaml";
    string intrinsic_parameters_path = ros::package::getPath("ips") + "/cfg/cameras/" + camera.getSerialNumber() + "/intrinsic_parameters.yaml";

    CameraParameters params;

    params.setExtrinsicParametersFromYAML(extrinsic_parameters_path);
    params.setIntrinsicParametersFromYAML(intrinsic_parameters_path);

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


    int active_crop_detection_jobs = 0;
    cv::Mat previous_image;
    while(loop)
    {

        // Get new image
        WithTimestamp<cv::Mat> image;
        if(!camera.grabImage(image)) {
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

        // convert detections to 3D rays and show intersection with the plane z=0
        for (auto const &detection:detection_per_vehicle) {
            if(detection) {
                auto point = detection.value().points[AprilTagDetection::i_center];

                cv::Vec3d origin;
                cv::Mat3d directions;
                tie(origin, directions) = params.pixelRays(cv::Mat2d(1,1, cv::Vec2d(point[0], point[1])));
                cv::Vec3d direction = directions(0,0);

                double scale_factor = -origin(2) / direction(2);
                cv::Vec3d floor_intersection_point = origin + direction * scale_factor;
                cout
                    << "id " << detection.value().id
                    << "  x " << std::setw( 11 ) << std::setfill(' ') << std::fixed << std::setprecision( 3 ) << floor_intersection_point(0)
                    << "  y " << std::setw( 11 ) << std::setfill(' ') << std::fixed << std::setprecision( 3 ) << floor_intersection_point(1)
                    << endl;
            }
        }


        // visualize
        if(previous_image.rows > 0 && previous_image.cols > 0) {
            cv::cvtColor(previous_image, previous_image, CV_GRAY2BGR);

            for (auto const &detection:detection_per_vehicle) {
                if(detection) {
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

            cv::imshow(camera.getSerialNumber(), previous_image);

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
    vector<string> camera_serial_numbers { "21967260" /* , "21704342" */ };
    vector<CameraWrapper> cameras;

    for (auto serial_no: camera_serial_numbers) {
        try {
            cameras.emplace_back(serial_no);
        }
        catch (const std::runtime_error &e) {
            cout << e.what() << endl;
            return 1;
        }
    }

    for (auto &camera: cameras) {
        camera.setGainExposure(5,1000);
    }

    ros::Time::init();

    vector<thread> grabThreads;
    for (auto &camera: cameras) {
        grabThreads.emplace_back([&](){grabLoop(camera);});
    }

    // trigger Loop
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while(loop) {
        for (size_t i = 0; i < cameras.size(); ++i) {
            cameras[i].triggerExposure();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    for(auto &t:grabThreads) t.join();
    cameras.clear();

}