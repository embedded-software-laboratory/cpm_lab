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
    //TODO: Find out what orientation exactly means, here it seems to apply to shapes as well (see scenario server (commonroad))
    if(orientation.has_value())
    {
        if(orientation->is_exact())
        {
            ctx->rotate(orientation->get_exact_value().value());
            position->draw(ctx, scale);
        }
        else
        {
            //Try to draw the position for every possible middle value of the orientation
            //TODO: Find out how to properly draw interval values
            if (orientation->get_interval().has_value())
            {
                for (auto &middle : orientation->get_interval()->get_interval_avg())
                {
                    //Draw position
                    ctx->save();
                    ctx->rotate(middle);
                    position->draw(ctx, scale);
                    ctx->restore();

                    //Draw arrow - TODO: Maybe make this a utility function
                    ctx->save();
                    ctx->rotate(middle);
                    position->transform_context(ctx, scale);
                    double arrow_scale = 0.3; //To quickly change the scale to your liking
                    ctx->set_line_width(0.015 * arrow_scale);
                    ctx->move_to(0.0, 0.0);
                    ctx->line_to(1.0 * arrow_scale, 0.0);
                    ctx->line_to(0.9 * arrow_scale, 0.1 * arrow_scale);
                    ctx->line_to(0.9 * arrow_scale, -0.1 * arrow_scale);
                    ctx->line_to(1.0 * arrow_scale, 0.0);
                    ctx->fill_preserve();
                    ctx->stroke();
                    ctx->restore();
                }
            }
            else
            {
                std::cerr << "TODO: Better warning // No orientation value (exact or interval) found for drawing" << std::endl;
            }
            
        }
    }

    //TODO: Draw velocity etc?

    ctx->restore();
}

void State::transform_context(const DrawingContext& ctx, double scale)
{
    //Draw at the stored position (if possible), else somehow within the possible position
    position->transform_context(ctx, scale);
    
    //Rotate, if necessary
    if(orientation.has_value())
    {
        if(orientation->is_exact())
        {
            ctx->rotate(orientation->get_exact_value().value());
        }
        else
        {
            std::cerr << "TODO: Better warning // State orientation is an interval - ignoring orientation" << std::endl;
        }
        
    }
}