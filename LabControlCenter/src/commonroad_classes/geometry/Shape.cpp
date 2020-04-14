#include "commonroad_classes/geometry/Shape.hpp"

Shape::Shape(const xmlpp::Node* node)
{
    //TODO: Assert node name to be position

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

void Shape::draw(const DrawingContext& ctx, double scale) 
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