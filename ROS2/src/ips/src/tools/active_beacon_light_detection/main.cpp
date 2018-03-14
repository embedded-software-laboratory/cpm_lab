#include "cpm_tools/default.hpp"
#include "cpm_tools/ThreadSafeQueue.hpp"
#include <opencv2/opencv.hpp>
#include "CameraWrapper/CameraWrapper.hpp"
#include "CameraParameters/CameraParameters.hpp"
#include "cpm_msgs/msg/vehicle_observation.hpp"

struct DetectionVisualizationInfo {
    vector<cv::Point2f> detections;
    cv::Mat image;
    string camera_serial_number;
    shared_ptr<CameraParameters> cameraParameters;
};

std::array<double, 2> floor_projection_point(std::array<double, 2> point,
    shared_ptr<CameraParameters> cameraParameters) {

    cv::Vec3d origin;
    cv::Mat3d directions;
    tie(origin, directions) = cameraParameters->pixelRays(cv::Mat2d(1,1, cv::Vec2d(point[0], point[1])));
    cv::Vec3d direction = directions(0,0);

    double scale_factor = -origin(2) / direction(2);
    cv::Vec3d floor_intersection_point = origin + direction * scale_factor;

    return { floor_intersection_point(0) ,floor_intersection_point(1)};

}

struct Cluster {
    float x_sum = 0;
    float y_sum = 0;
    float weight = 0;
};

vector<cv::Point2f> detect_light_blobs(cv::Mat img) {

    cv::Mat image_bw;
    cv::threshold(img,image_bw, 127, 255, cv::THRESH_BINARY);

    vector<cv::Point> locations;
    vector<Cluster> clusters;
    cv::findNonZero(image_bw, locations);
    for(const auto& location:locations) {
        bool insert_new_cluster = true;
        for(auto& cluster:clusters) {
            auto dx = location.x - (cluster.x_sum / cluster.weight);
            auto dy = location.y - (cluster.y_sum / cluster.weight);
            if(dx*dx + dy*dy < 8*8) {
                cluster.x_sum += location.x;
                cluster.y_sum += location.y;
                cluster.weight += 1;
                insert_new_cluster = false;
                break;
            }
        }
        if(insert_new_cluster) {
            Cluster c;
            c.x_sum = location.x;
            c.y_sum = location.y;
            c.weight = 1;
            clusters.push_back(c);
        }
    }


    vector<cv::Point2f> detections;
    for(auto& cluster:clusters)
    {
        if(3 <= cluster.weight && cluster.weight < 200) {
            float x = cluster.x_sum / cluster.weight;
            float y = cluster.y_sum / cluster.weight;
            detections.emplace_back(x,y);
        }
    }
    return detections;
}



using namespace cpm_tools;

class IpsNode : public CpmNode {    
    rclcpp::Publisher<cpm_msgs::msg::VehicleObservation>::SharedPtr publisher_;
    vector<string> camera_serial_numbers;
    vector<shared_ptr<CameraWrapper>> cameras;
    vector<shared_ptr<CameraParameters>> camera_parameters;
    shared_ptr<ThreadSafeQueue< DetectionVisualizationInfo > > visualization_queue;
    thread visualization_thread;

public:
    IpsNode() : CpmNode("IpsNode", 40 * NANOSEC_PER_MILLISEC, 0, false)
    {
        publisher_ = this->create_publisher<cpm_msgs::msg::VehicleObservation>("vehicle04/observation", rmw_qos_profile_sensor_data);

        camera_serial_numbers = { "22511669" /* , "21704342" */ };

        for (auto serial_no: camera_serial_numbers) {
            try {
                shared_ptr<CameraWrapper> cam = std::make_shared<CameraWrapper>(serial_no);
                cameras.push_back(cam);
            }
            catch (const std::runtime_error &e) {
                cout << e.what() << endl;
                exit(EXIT_FAILURE);
            }
        }

        for (auto &camera: cameras) {
            camera->setGainExposure(0,50);

            string extrinsic_parameters_path = "src/ips/cfg/cameras/" + camera->getSerialNumber() + "/extrinsic_parameters.yaml";
            string intrinsic_parameters_path = "src/ips/cfg/cameras/" + camera->getSerialNumber() + "/intrinsic_parameters.yaml";

            shared_ptr<CameraParameters> params = std::make_shared<CameraParameters>();

            params->setExtrinsicParametersFromYAML(extrinsic_parameters_path);
            params->setIntrinsicParametersFromYAML(intrinsic_parameters_path);
            camera_parameters.push_back(params);
        }


        visualization_queue = std::make_shared<ThreadSafeQueue< DetectionVisualizationInfo >>();
        visualization_thread = std::thread([this](){this->visualization_loop(this->visualization_queue);});
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        for (size_t i = 0; i < cameras.size(); ++i) {
            cameras[i]->triggerExposure();
        }

        for (size_t i = 0; i < cameras.size(); ++i) {
            // TODO parallelize this loop
            auto &camera = cameras[i];
            auto &camera_parameter = camera_parameters[i];
            processFrame(camera, camera_parameter, deadline_nanoseconds);
        }
    }

    void processFrame(shared_ptr<CameraWrapper> camera, shared_ptr<CameraParameters> params, uint64_t deadline_nanoseconds) {        

        cv::Mat image;
        if(!camera->grabImage(image)) {
            cout << "grabImage() failed" << endl;
            rclcpp::shutdown();
            return;
        }

        vector<cv::Point2f> detected_light_blobs = detect_light_blobs(image);

        vector<std::array<double, 2>> floor_points;
        for(auto p:detected_light_blobs) floor_points.push_back(floor_projection_point({p.x, p.y},params));

        //for(auto p:floor_points) { cout << "(" << p[0] << ", " << p[1] << "),  "; } cout << endl;


        // Calculate pose for _single_ vehicle (special case).
        // TODO pose detection and identification for multiple vehicles.
        if(floor_points.size() == 3) {
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
            
            auto message = cpm_msgs::msg::VehicleObservation();
            // calculate translation from "origin LED" to vehicle origin (= center of rear axis)
            message.pose.position.x = floor_points[origin_index][0] - 0.062 * cos(theta) + (-0.024) * sin(theta);
            message.pose.position.y = floor_points[origin_index][1] - 0.062 * sin(theta) - (-0.024) * cos(theta);

            message.pose.orientation.x = 0;
            message.pose.orientation.y = 0;
            message.pose.orientation.z = sin(theta/2.0);
            message.pose.orientation.w = cos(theta/2.0);

            message.stamp_nanoseconds = deadline_nanoseconds;

            publisher_->publish(message);
        }

        //cout << detected_light_blobs.size() << endl;

        // Send visualization info
        {
            DetectionVisualizationInfo detectionVisualizationInfo;
            detectionVisualizationInfo.camera_serial_number = camera->getSerialNumber();
            detectionVisualizationInfo.detections = detected_light_blobs;
            detectionVisualizationInfo.cameraParameters = params;
            detectionVisualizationInfo.image = image.clone();
            visualization_queue->write_nonblocking(detectionVisualizationInfo);
        }
    }

    void visualization_loop(shared_ptr< ThreadSafeQueue< DetectionVisualizationInfo > > visualization_queue) {
        while(rclcpp::ok()) {
            DetectionVisualizationInfo info;
            if(!visualization_queue->read(info)) return;

            
            // visualize
            if(info.image.rows > 0 && info.image.cols > 0) {
                cv::cvtColor(info.image, info.image, CV_GRAY2BGR);

                for (auto const &detection: info.detections) {
                    cv::circle(info.image, detection, 8, cv::Scalar(0,0,255));
                }

                //cv::resize(info.image, info.image, cv::Size(), 0.5, 0.5);
                cv::imshow(info.camera_serial_number, info.image);

                int key = cv::waitKey(1);
                if (key == 27) {
                    rclcpp::shutdown();
                    return;
                }
            }
        }
    }

    ~IpsNode(){
        rclcpp::shutdown();
        for (auto &camera: cameras) camera->close();
        visualization_queue->close();
        visualization_thread.join();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IpsNode>();
    std::thread node_thread([&](){node->start_loop();});
    rclcpp::spin(node);
    rclcpp::shutdown();
    node_thread.join();
    return 0;
}