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

void Polygon::draw(const DrawingContext& ctx, double scale)
{
    if (points.size() < 3)
    {
        std::cerr << "TODO: Better warning // Points missing in translated polygon (at least 3 required) - will not be drawn" << std::endl;
    }
    else
    {
        ctx->save();
        ctx->set_line_width(0.005);

        //Move to first point
        ctx->move_to(points.at(0).get_x() * scale, points.at(0).get_y() * scale);

        //Draw lines to remaining points
        for (auto point : points)
        {
            ctx->line_to(point.get_x() * scale, point.get_y() * scale);
        }
        ctx->stroke();

        ctx->restore();
    }
}