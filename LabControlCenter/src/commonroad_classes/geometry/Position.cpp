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
    std::cerr << "TODO: Better warning // Default values of position not yet known in implementation - cannot translate properly without these right now" << std::endl;
}

void Position::draw(const DrawingContext& ctx, double scale)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();
    
    //Rotate, if necessary
    if(point.has_value())
    {
        point->draw(ctx, scale);
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot draw inexact position right now" << std::endl;
    }

    ctx->restore();
}

void Position::draw_shape(const DrawingContext& ctx, std::optional<Shape> shape, double scale)
{
    ctx->save();

    //Rotate, if necessary
    if(point.has_value())
    {
        ctx->translate(point->get_x() * scale, point->get_y() * scale);

        if (shape.has_value())
        {
            shape->draw(ctx, scale);
        }
        else
        {
            std::cerr << "TODO: Better warning // Cannot draw shape at position, no value set for shape" << std::endl;
        }
        
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot draw shape at inexact position right now" << std::endl;
    }

    ctx->restore();
}