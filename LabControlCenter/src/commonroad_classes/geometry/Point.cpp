#include "commonroad_classes/geometry/Point.hpp"

Point::Point(const xmlpp::Node* node) 
{
    //TODO: Assert node name to be point

    //2018 and 2020
    x = xml_translation::get_child_child_double(node, "x", true);
    y = xml_translation::get_child_child_double(node, "y", true);

    //2020
    z = xml_translation::get_child_child_double(node, "z", false);
    //TODO: Remember somehow if z was set at all? Is optional and 2020 only, but -1.0 is a valid value -> use std::optional

    //Test output
    std::cout << "New point created: " << "(" << x << ", " << y << ", " << z << ")" << std::endl;
}

Point::Point(int irrelevant_int)
{
    //TODO: Find out default value
}