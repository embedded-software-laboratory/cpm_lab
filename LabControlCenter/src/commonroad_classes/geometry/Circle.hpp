#pragma once

#include "commonroad_classes/geometry/Point.hpp"

/**
 * \class Circle
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Circle
{
private:
    Point center; //must not be set (then in ??)
    double radius; //TODO: In constructor: Check if >= 0, must be unsigned

public:
    //TODO: constructor

    //TODO: From interface
    void transform_to_lane_width(unsigned int width);
    void to_dds_msg(); 

    //TODO: Getter
};