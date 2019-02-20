#include "default.hpp"
#include "CameraWrapper.hpp"
#include <opencv2/imgproc/types_c.h>
#include <unistd.h>


uint64_t get_time()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

int main() {

    shared_ptr<CameraWrapper> camera = std::make_shared<CameraWrapper>("22511669");
    camera->setGainExposure(0, 700);


    while(true)
    {
        camera->triggerExposure();

        cv::Mat image;
        if(!camera->grabImage(image)) {
            cout << "grabImage() failed" << endl;
            return 1;
        }


        auto t = get_time()/1000000;
        char filename[1000];
        snprintf(filename, 999, "img_%lu.png", t);
        cv::imwrite(filename, image);
        printf("image \"%s\" saved\n", filename);


        //usleep(500000);
        sleep(3);
    }

    return 0;
}