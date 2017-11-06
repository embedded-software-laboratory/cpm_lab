#include "AprilTagDetector.h"

AprilTagDetector::AprilTagDetector(AprilTagFamily family):family(family) {

    if(family == AprilTagFamily::Tag25h7) tf = tag25h7_create();
    else if(family == AprilTagFamily::Tag16h5) tf = tag16h5_create();
    else if(family == AprilTagFamily::Tag25h9) tf = tag25h9_create();
    else if(family == AprilTagFamily::Tag36h10) tf = tag36h10_create();
    else tf = tag36h11_create();

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    tf->black_border = 1;
    td->quad_decimate = 1;
    td->quad_sigma = 0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;
    td->refine_decode = 0;
    td->refine_pose = 0;
}

AprilTagDetector::~AprilTagDetector() {
    apriltag_detector_destroy(td);

    if(family == AprilTagFamily::Tag25h7) tag25h7_destroy(tf);
    else if(family == AprilTagFamily::Tag16h5) tag16h5_destroy(tf);
    else if(family == AprilTagFamily::Tag25h9) tag25h9_destroy(tf);
    else if(family == AprilTagFamily::Tag36h10) tag36h10_destroy(tf);
    else tag36h11_destroy(tf);
}

std::vector<AprilTagDetection> AprilTagDetector::detect(cv::Mat1b image) {
    image_u8_t im = {
        .width = image.cols,
        .height = image.rows,
        .stride = int32_t(image.step.buf[0]),
        .buf = image.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    std::vector<AprilTagDetection> my_detections;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        my_detections.push_back(AprilTagDetection {
            .points = std::array<std::array<double, 2>, 5> {
                std::array<double, 2>{det->p[0][0], det->p[0][1]},
                std::array<double, 2>{det->p[1][0], det->p[1][1]},
                std::array<double, 2>{det->p[2][0], det->p[2][1]},
                std::array<double, 2>{det->p[3][0], det->p[3][1]},
                std::array<double, 2>{det->c[0], det->c[1]}
            },
            .id = det->id,
            .hamming = det->hamming,
            .goodness = det->goodness,
            .decision_margin = det->decision_margin
        });
    }

    apriltag_detections_destroy(detections);
    return my_detections;
};