#pragma once

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag25h7.h"

#include <array>
#include <vector>

struct AprilTagDetection {
    // Point indices
    static constexpr int i_bottom_left = 0;
    static constexpr int i_bottom_right = 1;
    static constexpr int i_top_right = 2;
    static constexpr int i_top_left = 3;
    static constexpr int i_center = 4;

    std::array<std::array<double, 2>, 5> points;
    int id;
    int hamming;
    float goodness;
    float decision_margin;
};

class AprilTagDetector {

    apriltag_family_t *tf;
    apriltag_detector_t *td;

public:
    AprilTagDetector(); // TODO argument for tag family type, eg "36h11"
    ~AprilTagDetector();

    std::vector<AprilTagDetection> detect(image_u8_t im);
};