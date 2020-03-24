#pragma once

#include <vector>

#include "commonroad_classes/geometry/Circle.hpp"
#include "commonroad_classes/geometry/Polygon.hpp"
#include "commonroad_classes/geometry/Rectangle.hpp"

/**
 * \class Shape
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Shape
{
private:
    std::vector<Circle> circles;
    std::vector<Polygon> polygons;
    std::vector<Rectangle> rectangles;

public:
    //TODO: constructor

    //TODO: Draw function, maybe given a cairo context
    void draw() {}

    //TODO: From interface
    void transform_to_lane_width(unsigned int width) {}
    void to_dds_msg() {}

    //TODO: Getter
};