#pragma once

#include <vector>

#include "commonroad_classes/geometry/Point.hpp"

/**
 * \class Polygon
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Polygon
{
private:
    std::vector<Point> points;

public:
    //TODO: constructor

    //TODO: From interface
    void transform_to_lane_width(unsigned int width);
    void to_dds_msg(); 

    //TODO: Getter
};