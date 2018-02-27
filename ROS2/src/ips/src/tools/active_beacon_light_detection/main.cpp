#include "cpm_tools/default.hpp"
#include "cpm_tools/ThreadSafeQueue.hpp"
#include <opencv2/opencv.hpp>
#include "CameraWrapper/CameraWrapper.hpp"
#include "CameraParameters/CameraParameters.hpp"
#include "cpm_tools/AbsoluteTimer.hpp"

bool loop = true;

struct DetectionVisualizationInfo {
    vector<cv::Point2f> detections;
    cv::Mat image;
    string camera_serial_number;
    shared_ptr<CameraParameters> cameraParameters;
};


std::array<double, 2> floor_projection_point(std::array<double, 2> point,
    DetectionVisualizationInfo &info) {

    cv::Vec3d origin;
    cv::Mat3d directions;
    tie(origin, directions) = info.cameraParameters->pixelRays(cv::Mat2d(1,1, cv::Vec2d(point[0], point[1])));
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

void processFrame(
    shared_ptr<CameraWrapper> camera,
    shared_ptr<CameraParameters> params,
    shared_ptr< ThreadSafeQueue< DetectionVisualizationInfo> > visualization_queue) {

    
    camera->triggerExposure();
    WithTimestamp<cv::Mat> image;
    if(!camera->grabImage(image)) {
        loop = false;
        return;
    }


    auto t1 = clock_gettime_nanoseconds();

    cv::Mat image_bw;
    cv::threshold(image,image_bw, 127, 255, cv::THRESH_BINARY);




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


    auto t2 = clock_gettime_nanoseconds();
    cout << "   dt: " << ((t2-t1)/1000000) << "   --  ";
    cout << detections.size();
    cout << endl;

    // Send visualization info
    {
        DetectionVisualizationInfo detectionVisualizationInfo;
        detectionVisualizationInfo.camera_serial_number = camera->getSerialNumber();
        detectionVisualizationInfo.detections = detections;
        detectionVisualizationInfo.cameraParameters = params;
        detectionVisualizationInfo.image = image.clone();
        visualization_queue->write_nonblocking(detectionVisualizationInfo);
    } 

}


void visualization_loop(shared_ptr< ThreadSafeQueue< DetectionVisualizationInfo > > visualization_queue) {
    while(loop) {
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
            if (key == 27) loop = false;
        }
    }
}


int main(int argc, char* argv[])
{
    vector<string> camera_serial_numbers { "22511669" /* , "21704342" */ };
    vector<shared_ptr<CameraWrapper>> cameras;

    for (auto serial_no: camera_serial_numbers) {
        try {
            cameras.push_back(make_shared<CameraWrapper>(serial_no));
        }
        catch (const std::runtime_error &e) {
            cout << e.what() << endl;
            return 1;
        }
    }

    vector<shared_ptr<CameraParameters>> camera_parameters;
    for (auto &camera: cameras) {
        camera->setGainExposure(0,50);

        string extrinsic_parameters_path = "src/ips/cfg/cameras/" + camera->getSerialNumber() + "/extrinsic_parameters.yaml";
        string intrinsic_parameters_path = "src/ips/cfg/cameras/" + camera->getSerialNumber() + "/intrinsic_parameters.yaml";

        shared_ptr<CameraParameters> params = make_shared<CameraParameters>();

        params->setExtrinsicParametersFromYAML(extrinsic_parameters_path);
        params->setIntrinsicParametersFromYAML(intrinsic_parameters_path);
        camera_parameters.push_back(params);
    }

    auto visualization_queue = make_shared<ThreadSafeQueue< DetectionVisualizationInfo >>();

    
    vector< shared_ptr< cpm_tools::AbsoluteTimer > > timers;

    for (size_t i = 0; i < cameras.size(); ++i) {
        auto &camera = cameras[i];
        auto &camera_parameter = camera_parameters[i];
        timers.push_back(make_shared<cpm_tools::AbsoluteTimer>(0, 100 * 1000000, 0, 0,[&](){
            processFrame(camera, camera_parameter, visualization_queue);
        }));
    }

    visualization_loop(visualization_queue);

    for (auto &camera: cameras) camera->close();
    visualization_queue->close();
}