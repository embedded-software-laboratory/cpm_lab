#include "commonroad_classes/states/Occupancy.hpp"

Occupancy::Occupancy(const xmlpp::Node* node)
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

void Occupancy::draw(const DrawingContext& ctx, double scale, double orientation, double translate_x, double translate_y)
{
    //TODO: Include time value
    //Draw shape
    if (shape.has_value())
    {
        shape->draw(ctx, scale);
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot draw occupancy, shape is missing" << std::endl;
    }
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