#include "commonroad_classes/states/State.hpp"

State::State(const xmlpp::Node* node)
{
    //TODO: Warn if node is not of type state/initialState etc

    //TODO: Full translation of this type (currently: Only parts necessary for simple drawing)
    //Only these three values are mandatory (TODO: Might have default values though)
    const auto position_node = xml_translation::get_child_if_exists(node, "position", true);
    if (position_node)
    {
        position = std::optional<Position>{std::in_place, position_node};
    }

    const auto orientation_node = xml_translation::get_child_if_exists(node, "orientation", true);
    if (orientation_node)
    {
        orientation = std::optional<IntervalOrExact>{std::in_place, orientation_node};
    }

    const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
    if (time_node)
    {
        time = std::optional<IntervalOrExact>{std::in_place, time_node};
    }
}

void State::draw(const DrawingContext& ctx, double scale)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();
    
    //Rotate, if necessary
    if(orientation.has_value())
    {
        ctx->rotate(orientation->get_exact_value());
    }

    position->draw(ctx, scale);

    ctx->restore();
}

void State::transform_context(const DrawingContext& ctx, double scale)
{
    //Draw at the stored position (if possible), else somehow within the possible position
    position->transform_context(ctx, scale);
    
    //Rotate, if necessary
    if(orientation.has_value())
    {
        ctx->rotate(orientation->get_exact_value());
    }
}