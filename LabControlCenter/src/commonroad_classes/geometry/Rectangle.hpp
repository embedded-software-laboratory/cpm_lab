#pragma once

#include "commonroad_classes/geometry/Point.hpp"

/**
 * \class Rectangle
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Rectangle
{
private:
    Point center;
    double length; //TODO: In constructor: Check if >= 0, must be unsigned
    double orientation;
    double width;  //TODO: In constructor: Check if >= 0, must be unsigned

public:
    //TODO: constructor

    //TODO: From interface
    void transform_to_lane_width(unsigned int width);
    void to_dds_msg(); 

    //TODO: Getter
};