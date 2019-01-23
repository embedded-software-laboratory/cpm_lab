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
    double& world_y
)
{ 
    image_x /= 2048;
    image_y /= 2048;

    double ix1 = image_x;
    double ix2 = ix1 * image_x;
    double ix3 = ix2 * image_x;
    double ix4 = ix3 * image_x;

    double iy1 = image_y;
    double iy2 = iy1 * image_y;
    double iy3 = iy2 * image_y;
    double iy4 = iy3 * image_y;

    double features[] = {  
        1, ix1, iy1, ix2, ix1 * iy1, iy2, ix3, ix2 * iy1, ix1 * iy2, iy3,
        ix4, ix3 * iy1, ix2 * iy2, ix1 * iy3, iy4};

    double calibration_x[] = {4.351151e+00, -4.863260e+00, -1.183271e-01, 1.094383e+00, 4.861627e-01, 1.418471e-01, -6.700942e-01, 3.049470e-01, -4.362105e-01, 1.852765e-01, -7.676803e-02, -7.289893e-02, -1.313038e-01, -8.945004e-02, -5.608838e-02};
    double calibration_y[] = {-3.409322e-01, 5.226075e-01, 4.909991e+00, -3.901540e-01, -5.209485e-01, -1.093096e+00, 6.016393e-02, 5.101132e-01, -3.097603e-01, 5.542172e-01, -2.875253e-02, 5.073364e-02, 1.783163e-01, 1.005458e-01, 1.114947e-01};


    world_x = 0;
    world_y = 0;
    for (int i = 0; i < 15; ++i)
    {
        world_x += features[i] * calibration_x[i];
        world_y += features[i] * calibration_y[i];
    }
}


int main() {

    shared_ptr<CameraWrapper> camera = std::make_shared<CameraWrapper>("22511669");
    camera->setGainExposure(0, 700);


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