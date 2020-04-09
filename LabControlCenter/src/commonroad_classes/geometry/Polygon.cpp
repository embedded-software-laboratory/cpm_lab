#include "commonroad_classes/geometry/Polygon.hpp"

Polygon::Polygon(const xmlpp::Node* node)
{
    xml_translation::iterate_children(
        node,
        [&] (const xmlpp::Node* child)
        {
            points.push_back(Point(child));
        },
        "point"
    );

    if (points.size() < 3)
    {
        std::cerr << "TODO: Better warning // Points missing in translated polygon (at least 3 required): " << node->get_name() << "; line: " << node->get_line() << std::endl;
    }

    //Test output
    std::cout << "Polygon:" << std::endl;
    std::cout << "\tPoints: " << points.size() << std::endl;
}