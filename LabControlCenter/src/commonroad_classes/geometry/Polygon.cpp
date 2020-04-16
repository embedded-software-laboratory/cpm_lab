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
        ctx->line_to(points.at(0).get_x() * scale, points.at(0).get_y() * scale); //Finish polygon by drawing a line to the starting point
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
    }
}

const Point Polygon::get_center()
{
    //This is just the centroid, might not work for all shapes - TODO: Use a more complex algorithm? Might not really be worth the effort, also because it might take too long regarding the refresh time
    double sum_x = 0;
    double sum_y = 0;

    for (auto point : points)
    {
        sum_x += point.get_x();
        sum_y += point.get_y();
    }

    return Point(sum_x / static_cast<double>(points.size()), sum_y / static_cast<double>(points.size()), 0);
}