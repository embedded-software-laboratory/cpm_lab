#include "commonroad_classes/states/State.hpp"

State::State(const xmlpp::Node* node)
{
    //TODO: Warn if node is not of type state/initialState etc

    try
    {
        //2018 and 2020
        //Only these three values are mandatory (TODO: Might have default values though)
        const auto position_node = xml_translation::get_child_if_exists(node, "position", true);
        if (position_node)
        {
            position = std::optional<Position>{std::in_place, position_node};
        }

        orientation = get_interval(node, "orientation", true);
        time = get_interval(node, "time", true);

        //Not mandatory
        velocity = get_interval(node, "velocity", false);
        acceleration = get_interval(node, "acceleration", false);
        yaw_rate = get_interval(node, "yawRate", false);
        slip_angle = get_interval(node, "slipAngle", false);

        //2020 only and not mandatory
        steering_angle = get_interval(node, "steeringAngle", false);
        roll_angle = get_interval(node, "rollAngle", false);
        roll_rate = get_interval(node, "rollRate", false);
        pitch_angle = get_interval(node, "pitchAngle", false);
        pitch_rate = get_interval(node, "pitchRate", false);
        velocity_y = get_interval(node, "velocityY", false);
        position_z = get_interval(node, "positionZ", false);
        velocity_z = get_interval(node, "velocityZ", false);
        roll_angle_front = get_interval(node, "rollAngleFront", false);
        roll_rate_front = get_interval(node, "rollRateFront", false);
        velocity_y_front = get_interval(node, "velocityYFront", false);
        position_z_front = get_interval(node, "positionZFront", false);
        velocity_z_front = get_interval(node, "velocityZFront", false);
        roll_angle_rear = get_interval(node, "rollAngleRear", false);
        roll_rate_rear = get_interval(node, "rollRateRear", false);
        velocity_y_rear = get_interval(node, "velocityYRear", false);
        position_z_rear = get_interval(node, "positionZRear", false);
        velocity_z_rear = get_interval(node, "velocityZRear", false);
        left_front_wheel_angular_speed = get_interval(node, "leftFrontWheelAngularSpeed", false);
        right_front_wheel_angular_speed = get_interval(node, "rightFrontWheelAngularSpeed", false);
        left_rear_wheel_angular_speed = get_interval(node, "leftRearWheelAngularSpeed", false);
        right_rear_wheel_angular_speed = get_interval(node, "rightRearWheelAngularSpeed", false);
        delta_y_front = get_interval(node, "deltaYFront", false);
        delta_y_rear = get_interval(node, "deltaYRear", false);
        curvature = get_interval(node, "curvature", false);
        curvature_change = get_interval(node, "curvatureChange", false);
        jerk = get_interval(node, "jerk", false);
        jounce = get_interval(node, "jounce", false);
    }
    catch(const std::exception& e)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        //TODO: If desired, add "addInfo" function to error class to provide additional information
        throw;
    }
    
}

std::optional<IntervalOrExact> State::get_interval(const xmlpp::Node* node, std::string child_name, bool warn)
{
    const auto child_node = xml_translation::get_child_if_exists(node, child_name, warn);
    if (child_node)
    {
        return std::optional<IntervalOrExact>{std::in_place, child_node};
    }

    return std::optional<IntervalOrExact>();
}

void State::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    //TODO: Check if that's all
    
    if (position.has_value())
    {
        position->transform_coordinate_system(scale, translate_x, translate_y);
    }

    if (position_z.has_value())
    {
        position_z->transform_coordinate_system(scale, 0.0, 0.0);
    }

    if (position_z_front.has_value())
    {
        position_z_front->transform_coordinate_system(scale, 0.0, 0.0);
    }

    if (position_z_rear.has_value())
    {
        position_z_rear->transform_coordinate_system(scale, 0.0, 0.0);
    }

    if (delta_y_front.has_value())
    {
        delta_y_front->transform_coordinate_system(scale, 0.0, 0.0);
    }

    if (delta_y_rear.has_value())
    {
        delta_y_rear->transform_coordinate_system(scale, 0.0, 0.0);
    }

    if (scale > 0)
    {
        transform_scale *= scale;
    }
}

void State::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    if(!orientation.has_value())
    {
        std::cerr << "TODO: Better warning // No orientation value (exact or interval) found for drawing state, but should have been set - not drawing this state" << std::endl;
        return;
    }

    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);
    
    //Rotate, if necessary
    //TODO: Find out what orientation exactly means, here it seems to apply to shapes as well (see scenario server (commonroad))
    if(orientation->is_exact())
    {
        position->draw(ctx, scale, 0, 0, 0, orientation->get_exact_value().value() + local_orientation);
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
                position->draw(ctx, scale, 0, 0, 0, middle + local_orientation);
                ctx->restore();

                //Draw arrow - TODO: Maybe make this a utility function
                ctx->save();

                //Draw arrow
                position->transform_context(ctx, scale);
                ctx->rotate(middle + local_orientation);
                double arrow_scale = 0.6 * scale * transform_scale; //To quickly change the scale to your liking
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

    //TODO: Draw velocity etc?

    ctx->restore();
}

void State::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double, double)> _draw_lanelet_refs)
{
    if(position.has_value())
    {
        position->set_lanelet_ref_draw_function(_draw_lanelet_refs);
    }
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