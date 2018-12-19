#include "detect_light_blobs.hpp"

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