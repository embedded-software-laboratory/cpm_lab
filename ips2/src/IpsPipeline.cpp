#include "IpsPipeline.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>


IpsPipeline::IpsPipeline()
{
    undistortPointsFn = std::make_shared<UndistortPoints>(
        std::vector<double>{4.641747e+00, -5.379232e+00, -3.469735e-01, 1.598328e+00, 9.661605e-01, 3.870296e-01, -1.125387e+00, -1.264416e-01, -9.323793e-01, 5.223107e-02, 5.771384e-02, 7.367979e-02, 5.512993e-02, 3.857936e-02, -2.401879e-02},
        std::vector<double>{-5.985142e-01, 6.235073e-01, 5.412047e+00, -6.668763e-01, -1.038656e+00, -1.593830e+00, 2.284258e-01, 1.090997e+00, 9.839002e-02, 1.065165e+00, -8.319799e-02, -8.929741e-02, -5.524760e-02, -2.687517e-02, -5.680051e-02}
    );

    VehiclePointSet dummy;
    detectVehiclesFn = std::make_shared<DetectVehicles>(dummy);


    detectVehicleIDfn = std::make_shared<DetectVehicleID>(
        std::vector<uint8_t> { 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 },
        std::vector<uint8_t>{ 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 }
    );

    poseCalculationFn = std::make_shared<PoseCalculation>( );
}


void IpsPipeline::apply(LedPoints led_points)
{

    VehiclePoints identifiedVehicles;
    std::vector<VehicleObservation> vehicleObservations;


    FloorPoints floorPoints = undistortPointsFn->apply(led_points);
    VehiclePoints vehiclePoints = detectVehiclesFn->apply(floorPoints);
    vehiclePointTimeseries.push_back(vehiclePoints);
    if(vehiclePointTimeseries.size() > 50)
    {
        vehiclePointTimeseries.pop_front();
        identifiedVehicles = detectVehicleIDfn->apply(vehiclePointTimeseries);
        vehicleObservations = poseCalculationFn->apply(identifiedVehicles);
    }





    // Visualization
    {
        const int vis_img_width = 1700;
        const int vis_img_height = 1200;
        const double image_zoom_factor = 290;
        cv::Mat image(vis_img_height, vis_img_width, CV_8UC3, cv::Scalar(255,255,255));

        // Transformation from floor coordinates
        // to visualization image coordiantes.
        auto transform = [=](cv::Point2d floor_point)->cv::Point{
            return cv::Point(
                (floor_point.x - 2.25) * image_zoom_factor + vis_img_width/2,
                vis_img_height/2 - ((floor_point.y - 2.0) * image_zoom_factor)
            );
        };

        // Draw outline box
        cv::line(image, transform(cv::Point2d(0, 0)), transform(cv::Point2d(4.5, 0)),cv::Scalar(0,0,0),3);
        cv::line(image, transform(cv::Point2d(4.5, 4.0)), transform(cv::Point2d(4.5, 0)),cv::Scalar(0,0,0),3);
        cv::line(image, transform(cv::Point2d(4.5, 4.0)), transform(cv::Point2d(0, 4.0)),cv::Scalar(0,0,0),3);
        cv::line(image, transform(cv::Point2d(0,0)), transform(cv::Point2d(0, 4.0)),cv::Scalar(0,0,0),3);


        // Draw vehicle coordinate system
        for(const auto &vehicleObservation:vehicleObservations)
        {
            const double c = cos(vehicleObservation.pose().yaw());
            const double s = sin(vehicleObservation.pose().yaw());
            const double x = vehicleObservation.pose().x();
            const double y = vehicleObservation.pose().y();

            cv::arrowedLine(image,
                transform(cv::Point2d(x,y)),
                transform(cv::Point2d(x+0.2*c,y+0.2*s)),
                cv::Scalar(255,0,255),
                2,
                cv::LINE_AA
            );

            cv::arrowedLine(image,
                transform(cv::Point2d(x,y)),
                transform(cv::Point2d(x-0.2*s,y+0.2*c)),
                cv::Scalar(255,0,255),
                2,
                cv::LINE_AA
            );
        }

        // Draw floor points
        for(auto point : floorPoints.points)
        {
            cv::circle(image,transform(point),3,cv::Scalar(0,0,255),-1,cv::LINE_AA);
        }

        // Draw circles around detected vehicles
        for(auto vehicle:vehiclePoints.vehicles)
        {
            cv::circle(image,transform(
                0.5*vehicle.front + 0.25*vehicle.back_left + 0.25*vehicle.back_right
                ),0.1 * image_zoom_factor,cv::Scalar(0,128,255),1,cv::LINE_AA);
        }


        // Draw detected vehicle points
        for(auto vehicle:vehiclePoints.vehicles)
        {
            cv::putText(image,"L",transform(vehicle.back_left) + cv::Point(-3,5),cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(0,0,0),1);
            cv::putText(image,"R",transform(vehicle.back_right) + cv::Point(-3,5),cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(0,0,0),1);
            cv::putText(image,"C",transform(vehicle.center) + cv::Point(-3,5),cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(0,0,0),1);
            cv::putText(image,"F",transform(vehicle.front) + cv::Point(-3,5),cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(0,0,0),1);
        }


        // Draw vehcle IDs
        for(auto vehicle:identifiedVehicles.vehicles)
        {
            cv::putText(
                image,
                std::to_string(vehicle.id),
                transform(0.5*vehicle.front + 0.25*vehicle.back_left + 0.25*vehicle.back_right) + cv::Point(-20,10),
                cv::FONT_HERSHEY_SIMPLEX,
                2,
                cv::Scalar(0,180,0),
                3
            );
        }


        // Show image
        cv::imshow("IPS Visualization", image);
        if(cv::waitKey(1) == 27) // close on escape key
        {
            system("killall rtireplay");
            exit(0);
        }

    }
}
