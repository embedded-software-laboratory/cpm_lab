#include "default.hpp"
#include "CameraWrapper.hpp"
#include <opencv2/imgproc/types_c.h>

int main(int argc, char* argv[]) {

    shared_ptr<CameraWrapper> camera = std::make_shared<CameraWrapper>("22511669");
    camera->setGainExposure(0, 1000);

    while(1)
    {
        camera->triggerExposure();

        cv::Mat image;
        if(!camera->grabImage(image)) {
            cout << "grabImage() failed" << endl;
            return 0;
        }

        // visualize
        if(image.rows > 0 && image.cols > 0) {
            cv::cvtColor(image, image, CV_GRAY2BGR);

            cv::resize(image, image, cv::Size(), 0.5, 0.5);
            cv::imshow("22511669", image);

            int key = cv::waitKey(1);
            if (key == 27) {
                return 0;
            }
        }
    }

    return 0;
}