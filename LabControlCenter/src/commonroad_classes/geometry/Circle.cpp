#include "commonroad_classes/geometry/Circle.hpp"

Circle::Circle(const xmlpp::Node* node)
{
    //TODO: Check if node is of type circle

    try
    {
        radius = xml_translation::get_child_child_double(node, "radius", true); //mandatory

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
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Circle:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    //Test output
    std::cout << "Circle:" << std::endl;
    std::cout << "\tRadius: " << radius << std::endl;
    std::cout << "\tCenter set: " << center.has_value() << std::endl;
}

void Circle::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (scale > 0)
    {
        radius *= scale;
    }
    center->transform_coordinate_system(scale, translate_x, translate_y);
}

void Circle::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    //Local rotation does not really make sense here and is thus ignored (rotating a circle in its own coordinate system is pointless)
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.005);

    //Move to center
    ctx->move_to(center->get_x() * scale, center->get_y() * scale);

    //Draw circle
    ctx->arc(center->get_x() * scale, center->get_y() * scale, radius * scale, 0.0, 2 * M_PI);
    ctx->stroke();

    ctx->restore();
}

const std::optional<Point>& Circle::get_center() const
{
    return center;
}