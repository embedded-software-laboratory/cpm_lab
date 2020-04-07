#pragma once

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Point.hpp"

/**
 * \class Rectangle
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Rectangle
{
private:
    std::optional<Point> center; //must not be set (then in ??)
    double length; //TODO: In constructor: Check if >= 0, must be unsigned
    double orientation; //must not be set (then ??)
    double width;  //TODO: In constructor: Check if >= 0, must be unsigned

public:
    //TODO: constructor

    //TODO: From interface
    void transform_to_lane_width(unsigned int width) {}
    void to_dds_msg() {}

    //TODO: Getter
};