#include "commonroad_classes/states/GoalState.hpp"

GoalState::GoalState(const xmlpp::Node* node)
{
    //2018 and 2020 specs are the same
    //TODO: Assert node "type"

    const auto position_node = xml_translation::get_child_if_exists(node, "position", false);
    if (position_node)
    {
        position = std::optional<Position>{std::in_place, position_node};
    }
    else
    {
        //TODO: Check if default position value is spec-conform if no value is specified here
        //Use default-value constructor (parameter is irrelevant)
        position = std::optional<Position>{std::in_place, 0};
    }

    const auto velocity_node = xml_translation::get_child_if_exists(node, "velocity", false);
    if (velocity_node)
    {
        velocity = std::optional<Interval>(std::in_place, velocity_node);
    }

    const auto orientation_node = xml_translation::get_child_if_exists(node, "orientation", false);
    if (orientation_node)
    {
        orientation = std::optional<Interval>(std::in_place, orientation_node);
    }

    //Time is defined using intervals
    const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
    if (time_node)
    {
        time = std::optional<IntervalOrExact>(std::in_place, time_node);
    }
    else
    {
        //Time is the only actually required value
        std::cerr << "TODO: Better warning // No time node in GoalState - Line: " << node->get_line() << std::endl;
    }
    

    //Test output
    std::cout << "GoalState: " << std::endl;
    std::cout << "\tPosition exists: " << position.has_value() << std::endl;
    std::cout << "\tVelocity exists: " << velocity.has_value() << std::endl;
    std::cout << "\tOrientation exists: " << orientation.has_value() << std::endl;
    std::cout << "\tTime exists: " << time.has_value() << std::endl;
}

void GoalState::draw(const DrawingContext& ctx, double scale)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();
    
    //Rotate, if necessary
    if(orientation.has_value())
    {
        //Rotation is an interval - draw position for every possible orientation start and end value
        for (auto rot_it = orientation->cbegin(); rot_it != orientation->cend(); ++rot_it)
        {
            ctx->save();
            //ctx->rotate(rot_it->first);
            if(position.has_value())
            {
                position->draw(ctx, scale);
            }
            ctx->restore();

            ctx->save();
            //ctx->rotate(rot_it->second); -> TODO: Find out what orientation exactly means, probably only the vehicle orientation within the area -> only draw an arrow, don't rotate the shape
            if(position.has_value())
            {
                position->draw(ctx, scale);
            }
            ctx->restore();
        }
    }
    else
    {
        //Draw without rotating
        if(position.has_value())
        {
            position->draw(ctx, scale);
        }
    }

    //TODO: Draw velocity/time?
    //Also TODO: Test output for other state classes

    ctx->restore();
}