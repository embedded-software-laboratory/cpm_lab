#include "commonroad_classes/geometry/Polygon.hpp"

Polygon::Polygon(const xmlpp::Node* node)
{
    //TODO: Check if node is of type polygon

    try
    {
        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                points.push_back(Point(child));
            },
            "point"
        );
    }
    catch(const std::exception& e)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        //TODO: If desired, add "addInfo" function to error class to provide additional information
        throw;
    }

    if (points.size() < 3)
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Points missing in translated polygon (at least 3 required), line " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    //Test output
    std::cout << "Polygon:" << std::endl;
    std::cout << "\tPoints: " << points.size() << std::endl;
}

void Polygon::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    for (auto& point : points)
    {
        point.transform_coordinate_system(scale, translate_x, translate_y);
    }
}

void Polygon::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    if (points.size() < 3)
    {
        std::cerr << "TODO: Better warning // Points missing in translated polygon (at least 3 required) - will not be drawn" << std::endl;
    }
    else
    {
        ctx->save();

        //Perform required translation + rotation
        //TODO: Local transformation (Translate to center (use get_center()), rotate, draw from there using center-relative coordinates)
        ctx->translate(global_translate_x, global_translate_y);
        ctx->rotate(global_orientation);

        //To allow for rotation in the local coordinate system, move to the center, then rotate, then use relative positions (to the center)
        auto center = get_center();
        ctx->translate(center.get_x() * scale, center.get_y() * scale);
        ctx->rotate(local_orientation);
        
        ctx->set_line_width(0.005);

        //Move to first point
        ctx->move_to((points.at(0).get_x() - center.get_x()) * scale, (points.at(0).get_y() - center.get_y()) * scale);

        //Draw lines to remaining points
        for (auto point : points)
        {
            ctx->line_to((point.get_x() - center.get_x()) * scale, (point.get_y() - center.get_y()) * scale);
        }
        ctx->line_to((points.at(0).get_x() - center.get_x()) * scale, (points.at(0).get_y() - center.get_y()) * scale); //Finish polygon by drawing a line to the starting point
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