#include "default.hpp"
#include <opencv2/imgproc/types_c.h>
#include "detect_light_blobs.hpp"
#include "calibration_floor_plane.hpp"


int main() {

    string base_path = "/home/janis/Desktop/calibration_images/";

    vector<string> calibration_image_files {
        "08_11/0.png",
        "08_11/90.png",
        "08_11/315.png",
        "08_11/270.png",
        "08_11/180.png",
        "08_11/225.png",
        "08_11/135.png",
        "08_11/45.png",
        "35_22/0.png",
        "35_22/90.png",
        "35_22/315.png",
        "35_22/270.png",
        "35_22/180.png",
        "35_22/225.png",
        "35_22/135.png",
        "35_22/45.png",
        "07_29/0.png",
        "07_29/90.png",
        "07_29/315.png",
        "07_29/270.png",
        "07_29/180.png",
        "07_29/225.png",
        "07_29/135.png",
        "07_29/45.png",
    };

    const auto N = calibration_image_files.size();

    for (size_t i = 0; i < N; ++i)
    {
        int x_cm = 0;
        int y_cm = 0;
        int angle_deg = 0;
        sscanf(calibration_image_files[i].c_str(), "%d_%d/%d.png", &x_cm, &y_cm, &angle_deg);


        auto image = cv::imread(base_path + calibration_image_files[i]);
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
        auto light_blobs = detect_light_blobs(gray_image);


        printf("x: %d, y: %d, angle: %d, lights: %lu\n", x_cm, y_cm, angle_deg, light_blobs.size());
    }

    return 0;
}