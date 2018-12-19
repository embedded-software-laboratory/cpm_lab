#include "default.hpp"
#include "CameraWrapper.hpp"
#include <opencv2/imgproc/types_c.h>
#include "detect_light_blobs.hpp"
#include "cpm/Timer.hpp"
#include "VehicleObservation.hpp"
#include <dds/pub/ddspub.hpp>
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/stamp_message.hpp"


void calibration(
    double image_x,
    double image_y,
    double& world_x,
    double& wolrd_y
)
{
    // simple linear calibration for first tests
    world_x = 0.001924101682693 * image_x +   0.000051747462619 * image_y -0.637005868603361;
    wolrd_y = 0.000100025817748* image_x  -0.001941193807078 * image_y+   2.141834542011531;
}


int main() {

    shared_ptr<CameraWrapper> camera = std::make_shared<CameraWrapper>("22511669");
    camera->setGainExposure(0, 500);


    // DDS setup
    auto& participant = cpm::ParticipantSingleton::Instance();
    dds::topic::Topic<VehicleObservation> topic_vehicleObservation (
        participant, "vehicleObservation");
    dds::pub::DataWriter<VehicleObservation> writer_vehicleObservation(
        dds::pub::Publisher(participant), topic_vehicleObservation);




    auto update_loop = cpm::Timer::create("IPS", 80000000ull, 0);

    update_loop->start([&](uint64_t t_now) {
        camera->triggerExposure();

        cv::Mat image;
        if(!camera->grabImage(image)) {
            cout << "grabImage() failed" << endl;
            update_loop->stop();
        }

        vector<cv::Point2f> detected_light_blobs = detect_light_blobs(image);

        if(detected_light_blobs.size() == 3)
        {
            vector<std::array<double, 2>> floor_points;
            for(auto p:detected_light_blobs)
            {
                double x,y;
                calibration(p.x,p.y,x,y);
                floor_points.push_back({x,y});
            }

            // Calculate pose for _single_ vehicle (special case).
            double max_dist = 0;
            int max_dist_i = 0;
            for (int i = 0; i < 3; ++i) {
                double dx = floor_points[(i+1)%3][0] - floor_points[i][0];
                double dy = floor_points[(i+1)%3][1] - floor_points[i][1];
                double dist = sqrt(dx*dx + dy*dy);
                if(dist > max_dist) {
                    max_dist = dist;
                    max_dist_i = i;
                }
            }
            int origin_index = (max_dist_i + 2)%3;

            double dx1 = floor_points[(origin_index+1)%3][0] - floor_points[origin_index][0];
            double dy1 = floor_points[(origin_index+1)%3][1] - floor_points[origin_index][1];
            double dx2 = floor_points[(origin_index+2)%3][0] - floor_points[origin_index][0];
            double dy2 = floor_points[(origin_index+2)%3][1] - floor_points[origin_index][1];

            double theta = 0;
            if(dx1*dy2-dy1*dx2 > 0) {
                theta = atan2(dy1,dx1);
            }
            else {                
                theta = atan2(dy2,dx2);
            }

            theta = theta + 0.045;

            theta = remainder(theta, 2*M_PI);

            // calculate translation from "origin LED" to vehicle origin (= center of rear axis)
            double position_x = floor_points[origin_index][0] + (0.0) * cos(theta) + (0.0) * sin(theta);
            double position_y = floor_points[origin_index][1] + (0.0) * sin(theta) + (0.0) * cos(theta);

            cout << "pose (" << position_x << ", " << position_y << ", " << theta << ")" << endl;



            VehicleObservation vehicleObservation;
            vehicleObservation.vehicle_id(1);
            vehicleObservation.pose().x(position_x);
            vehicleObservation.pose().y(position_y);
            vehicleObservation.pose().yaw(theta);
            cpm::stamp_message(vehicleObservation, t_now, 0);
            writer_vehicleObservation.write(vehicleObservation);

            /*
            auto rem = t_now % 3000000000ull;
            for (uint8_t vehicle_id = 0; vehicle_id < 20; ++vehicle_id)
            {
                uint64_t slot = vehicle_id * 500000000ull;

                if( slot <= rem && rem < slot + 250000000ull )
                {
                    VehicleObservation vehicleObservation;
                    vehicleObservation.vehicle_id(vehicle_id);
                    vehicleObservation.pose().x(position_x);
                    vehicleObservation.pose().y(position_y);
                    vehicleObservation.pose().yaw(theta);
                    cpm::stamp_message(vehicleObservation, t_now, 0);
                    writer_vehicleObservation.write(vehicleObservation);
                    break;
                }
            }*/
        }

        // visualize
        if(image.rows > 0 && image.cols > 0) {
            cv::cvtColor(image, image, CV_GRAY2BGR);

            for (auto const &detection: detected_light_blobs) {
                cv::circle(image, detection, 8, cv::Scalar(0,0,255));
            }

            cv::resize(image, image, cv::Size(), 0.5, 0.5);
            cv::imshow("img", image);

            int key = cv::waitKey(1);
            if (key == 27) {
                update_loop->stop();
            }
        }
    });

    return 0;
}