#include "commonroad_classes/states/Occupancy.hpp"

Occupancy::Occupancy(const xmlpp::Node* node)
{
    //TODO: Check if node is of type occupancy

    try
    {
        const auto shape_node = xml_translation::get_child_if_exists(node, "shape", true);
        if (shape_node)
        {
            shape = std::optional<Shape>(std::in_place, shape_node);
        }
        
        const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
        if (time_node)
        {
            time = std::optional<IntervalOrExact>(std::in_place, time_node);
        }
    }
    catch(const std::exception& e)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        //TODO: If desired, add "addInfo" function to error class to provide additional information
        throw;
    }
    
}

void Occupancy::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (shape.has_value())
    {
        shape->transform_coordinate_system(scale, translate_x, translate_y);
    }
}

void Occupancy::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    //TODO: Include time value
    //Draw shape
    if (shape.has_value())
    {
        shape->draw(ctx, scale, 0, 0, 0, local_orientation);
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot draw occupancy, shape is missing" << std::endl;
    }

    ctx->restore();
}

void Occupancy::transform_context(const DrawingContext& ctx, double scale)
{
    //Transform to position of shape center, where the object can be drawn if desired
    if (shape.has_value())
    {
        shape->transform_context(ctx, scale);
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot transform context with occupancy, shape is missing" << std::endl;
    }
}