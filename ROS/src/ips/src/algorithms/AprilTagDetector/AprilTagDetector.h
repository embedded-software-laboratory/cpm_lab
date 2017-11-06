#pragma once

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag25h7.h"
#include "apriltag/tag16h5.h"

#include <array>
#include <vector>

/*!
    Represents the detection of a tag.
*/
struct AprilTagDetection {

    //@{
    /*! Index name for the AprilTagDetection::points array. */
    static constexpr int i_bottom_left = 0;
    static constexpr int i_bottom_right = 1;
    static constexpr int i_top_right = 2;
    static constexpr int i_top_left = 3;
    static constexpr int i_center = 4;
    //@}

    /*! (x,y) pixel coordinates for the tag corners and center. */
    std::array<std::array<double, 2>, 5> points;

    //@{
    /*! See <a href="file:///usr/local/include/apriltag/apriltag.h">apriltag.h</a>  */
    int id;
    int hamming;
    float goodness;
    float decision_margin;
    //@}
};

enum class AprilTagFamily { Tag36h11, Tag36h10, Tag25h9, Tag25h7, Tag16h5 };

/*!
    Thin wrapper class for the apriltag detector C library.
*/
class AprilTagDetector {

    apriltag_family_t *tf;
    apriltag_detector_t *td;
    AprilTagFamily family;

public:
    /*! \param family The used tag family. */
    AprilTagDetector(AprilTagFamily family);
    ~AprilTagDetector();

    /*! Find apriltags in a black and white image. */
    std::vector<AprilTagDetection> detect(image_u8_t im);
};