#include "commonroad_classes/geometry/Position.hpp"

Position::Position(const xmlpp::Node* node)
{
    //TODO: Assert node name to be position

    //2018 and 2020
    const auto point_node = xml_translation::get_child_if_exists(node, "point", false); //not mandatory
    if (point_node)
    {
        point = std::optional<Point>(std::in_place, point_node);
    }
    
    //Optional parts (all unbounded -> lists)
    xml_translation::iterate_children(
        node,
        [&] (const xmlpp::Node* child)
        {
            circles.push_back(Circle(child));
        },
        "circle"
    );

    xml_translation::iterate_children(
        node,
        [&] (const xmlpp::Node* child)
        {
            lanelet_refs.push_back(xml_translation::get_attribute_int(child, "ref", true));
        },
        "lanelet"
    );

    xml_translation::iterate_children(
        node,
        [&] (const xmlpp::Node* child)
        {
            polygons.push_back(Polygon(child));
        },
        "polygon"
    );

    xml_translation::iterate_children(
        node,
        [&] (const xmlpp::Node* child)
        {
            rectangles.push_back(Rectangle(child));
        },
        "rectangle"
    );

    //Test output
    std::cout << "Position:" << std::endl;
    std::cout << "\tPoint exists: " << point.has_value() << std::endl;
    std::cout << "\tCircle size: " << circles.size() << std::endl;
    std::cout << "\tLanelet ref size: " << lanelet_refs.size() << std::endl;
    std::cout << "\tPolygon size: " << polygons.size() << std::endl;
    std::cout << "\tRectangle size: " << rectangles.size() << std::endl;
}

Position::Position(int irrelevant_int)
{
    //TODO: Find out default value
}