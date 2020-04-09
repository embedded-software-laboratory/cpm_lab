#include "commonroad_classes/geometry/Rectangle.hpp"

Rectangle::Rectangle(const xmlpp::Node* node)
{
    length = xml_translation::get_child_child_double(node, "length", true); //mandatory
    width = xml_translation::get_child_child_double(node, "width", true); //mandatory

    //Get point value, which must not be specified
    const auto point_node = xml_translation::get_child_if_exists(node, "center", false);
    if (point_node)
    {
        center = std::optional<Point>{std::in_place, point_node};
    }
    else
    {
        //Use default-value constructor (parameter is irrelevant)
        center = std::optional<Point>{std::in_place, 0};
    }

    if (xml_translation::get_child_if_exists(node, "orientation", false))
    {
        orientation = xml_translation::get_child_child_double(node, "orientation", true);
    }
    else
    {
        //TODO: Find out default value
    }
    

    //Test output
    std::cout << "Rectangle:" << std::endl;
    std::cout << "\tLenght, width: " << length << ", " << width << std::endl;
    std::cout << "\tOrientation set: " << orientation.has_value() << std::endl;
    std::cout << "\tCenter set: " << center.has_value() << std::endl;
}