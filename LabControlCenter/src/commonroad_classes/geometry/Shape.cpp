#include "commonroad_classes/geometry/Shape.hpp"

Shape::Shape(const xmlpp::Node* node)
{
    //TODO: Assert node name to be shape

    //2018 and 2020    
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
    std::cout << "Shape:" << std::endl;
    std::cout << "\tCircle size: " << circles.size() << std::endl;
    std::cout << "\tPolygon size: " << polygons.size() << std::endl;
    std::cout << "\tRectangle size: " << rectangles.size() << std::endl;
}

void Shape::draw(const DrawingContext& ctx, double scale, double orientation, double translate_x, double translate_y) 
{
    ctx->save();

    ctx->set_line_width(0.005);

    for (auto circle : circles)
    {
        circle.draw(ctx, scale);
    }

    for (auto polygon : polygons)
    {
        polygon.draw(ctx, scale);
    }

    for (auto rectangle : rectangles)
    {
        rectangle.draw(ctx, scale);
    }

    ctx->restore();
}

void Shape::transform_context(const DrawingContext& ctx, double scale)
{
    //Just move the coordinate system's center to one of the shape's centroids
    //TODO: Find better point than just some random point within a part of the shape
    if (circles.size() > 0)
    {
        auto center = circles.at(0).get_center();
        if(center.has_value())
        {
            ctx->translate(center->get_x() * scale, center->get_y() * scale);
        }
    }
    else if (polygons.size() > 0)
    {
        auto center = polygons.at(0).get_center();
        //Center is computed from vertices, type is not std::optional and not const
        ctx->translate(center.get_x() * scale, center.get_y() * scale);
    }
    else if (rectangles.size() > 0)
    {
        auto center = rectangles.at(0).get_center();
        if(center.has_value())
        {
            ctx->translate(center->get_x() * scale, center->get_y() * scale);
        }
    }
}