// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "commonroad_classes/states/State.hpp"

State::State(const xmlpp::Node* node)
{
    //Check if node is of type state
    assert(node->get_name() == "state" || node->get_name() == "initialState");

    commonroad_line = node->get_line();

    try
    {
        //2018 and 2020
        //Only these three values are mandatory
        position = std::optional<Position>{std::in_place, xml_translation::get_child_if_exists(node, "position", true)};
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
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate State:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
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

void State::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    //TODO: Check if that's all
    
    if (position.has_value())
    {
        position->transform_coordinate_system(scale, angle, translate_x, translate_y);
    }

    if (position_z.has_value())
    {
        position_z->transform_coordinate_system(scale, angle, 0.0, 0.0);
    }

    if (position_z_front.has_value())
    {
        position_z_front->transform_coordinate_system(scale, angle, 0.0, 0.0);
    }

    if (position_z_rear.has_value())
    {
        position_z_rear->transform_coordinate_system(scale, angle, 0.0, 0.0);
    }

    if (delta_y_front.has_value())
    {
        delta_y_front->transform_coordinate_system(scale, angle, 0.0, 0.0);
    }

    if (delta_y_rear.has_value())
    {
        delta_y_rear->transform_coordinate_system(scale, angle, 0.0, 0.0);
    }

    //If all positional values are adjusted, the velocity must be adjusted as well
    if (velocity.has_value())
    {
        velocity->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (acceleration.has_value())
    {
        acceleration->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (velocity_y.has_value())
    {
        velocity_y->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (velocity_y_front.has_value())
    {
        velocity_y_front->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (velocity_y_rear.has_value())
    {
        velocity_y_rear->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (velocity_z.has_value())
    {
        velocity_z->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (velocity_z_front.has_value())
    {
        velocity_z_front->transform_coordinate_system(scale, angle, 0, 0);
    }
    if (velocity_z_rear.has_value())
    {
        velocity_z_rear->transform_coordinate_system(scale, angle, 0, 0);
    }

    if (scale > 0)
    {
        transform_scale *= scale;
    }
}

void State::transform_timing(double time_scale)
{
    if (velocity.has_value())
    {
        velocity->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (acceleration.has_value())
    {
        acceleration->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (velocity_y.has_value())
    {
        velocity_y->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (velocity_y_front.has_value())
    {
        velocity_y_front->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (velocity_y_rear.has_value())
    {
        velocity_y_rear->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (velocity_z.has_value())
    {
        velocity_z->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (velocity_z_front.has_value())
    {
        velocity_z_front->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (velocity_z_rear.has_value())
    {
        velocity_z_rear->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (left_front_wheel_angular_speed.has_value())
    {
        left_front_wheel_angular_speed->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (right_front_wheel_angular_speed.has_value())
    {
        right_front_wheel_angular_speed->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (left_rear_wheel_angular_speed.has_value())
    {
        left_rear_wheel_angular_speed->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (right_rear_wheel_angular_speed.has_value())
    {
        right_rear_wheel_angular_speed->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    if (yaw_rate.has_value())
    {
        yaw_rate->transform_coordinate_system(time_scale, 0, 0, 0);
    }
    
    //TODO: Scale curvature change as well?
}

void State::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    if(!orientation.has_value())
    {
        std::stringstream error_stream;
        error_stream << "Cannot draw state bc no orientation was set, from line " << commonroad_line;
        LCCErrorLogger::Instance().log_error(error_stream.str());
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
                ctx->set_source_rgba(.7,.2,.7,.2); //Color used for inexact values
                position->draw(ctx, scale, 0, 0, 0, middle + local_orientation);
                ctx->restore();

                //Draw arrow with correct position / orientation
                ctx->save();
                position->transform_context(ctx, scale);
                ctx->rotate(middle + local_orientation);

                double arrow_scale = scale * transform_scale; //To quickly change the scale to your liking
                draw_arrow(ctx, 0.0, 0.0, 2.0 * arrow_scale, 0.0, 2.0 * arrow_scale);

                ctx->restore();
            }
        }
        else
        {
            std::stringstream error_stream;
            error_stream << "No orientation value within one part of the set orientation intervals found for state, from line " << commonroad_line;
            LCCErrorLogger::Instance().log_error(error_stream.str());
        }
    }

    //TODO: Draw velocity etc?

    ctx->restore();
}

void State::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs)
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
            std::stringstream error_stream;
            error_stream << "Orientation from state for rotation of other objects ignored, as it is an interval value" << commonroad_line;
            LCCErrorLogger::Instance().log_error(error_stream.str());
        }
        
    }
}

//********************************************************************************************************************************************
//Getter
//********************************************************************************************************************************************

Position State::get_position()
{
    if (!position.has_value())
    {
        throw SpecificationError("State should have position value, but does not");
    }
    return position.value();
}

IntervalOrExact State::get_time()
{
    if (!time.has_value())
    {
        throw SpecificationError("State should have time value, but does not");
    }
    return time.value();
}

std::optional<double> State::get_orientation_mean()
{
    if(orientation.has_value())
    {
        return orientation.value().get_mean();
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<IntervalOrExact>& State::get_velocity()
{
    return velocity;
}

const std::optional<IntervalOrExact>& State::get_orientation() const
{
    return orientation;
}

const std::optional<IntervalOrExact>& State::get_acceleration() const
{
    return acceleration;
}

const std::optional<IntervalOrExact>& State::get_yaw_rate() const
{
    return yaw_rate;
}

const std::optional<IntervalOrExact>& State::get_slip_angle() const
{
    return slip_angle;
}

const std::optional<IntervalOrExact>& State::get_steering_angle() const
{
    return steering_angle;
}

const std::optional<IntervalOrExact>& State::get_roll_angle() const
{
    return roll_angle;
}

const std::optional<IntervalOrExact>& State::get_roll_rate() const
{
    return roll_rate;
}

const std::optional<IntervalOrExact>& State::get_pitch_angle() const
{
    return pitch_angle;
}

const std::optional<IntervalOrExact>& State::get_pitch_rate() const
{
    return pitch_rate;
}

const std::optional<IntervalOrExact>& State::get_velocity_y() const
{
    return velocity_y;
}

const std::optional<IntervalOrExact>& State::get_position_z() const
{
    return position_z;
}

const std::optional<IntervalOrExact>& State::get_velocity_z() const
{
    return velocity_z;
}

const std::optional<IntervalOrExact>& State::get_roll_angle_front() const
{
    return roll_angle_front;
}

const std::optional<IntervalOrExact>& State::get_roll_rate_front() const
{
    return roll_rate_front;
}

const std::optional<IntervalOrExact>& State::get_velocity_y_front() const
{
    return velocity_y_front;
}

const std::optional<IntervalOrExact>& State::get_position_z_front() const
{
    return position_z_front;
}

const std::optional<IntervalOrExact>& State::get_velocity_z_front() const
{
    return velocity_z_front;
}

const std::optional<IntervalOrExact>& State::get_roll_angle_rear() const
{
    return roll_angle_rear;
}

const std::optional<IntervalOrExact>& State::get_roll_rate_rear() const
{
    return roll_rate_rear;
}

const std::optional<IntervalOrExact>& State::get_velocity_y_rear() const
{
    return velocity_y_rear;
}

const std::optional<IntervalOrExact>& State::get_position_z_rear() const
{
    return position_z_rear;
}

const std::optional<IntervalOrExact>& State::get_velocity_z_rear() const
{
    return velocity_z_rear;
}

const std::optional<IntervalOrExact>& State::get_left_front_wheel_angular_speed() const
{
    return left_front_wheel_angular_speed;
}

const std::optional<IntervalOrExact>& State::get_right_front_wheel_angular_speed() const
{
    return right_front_wheel_angular_speed;
}

const std::optional<IntervalOrExact>& State::get_left_rear_wheel_angular_speed() const
{
    return left_rear_wheel_angular_speed;
}

const std::optional<IntervalOrExact>& State::get_right_rear_wheel_angular_speed() const
{
    return right_rear_wheel_angular_speed;
}

const std::optional<IntervalOrExact>& State::get_delta_y_front() const
{
    return delta_y_front;
}

const std::optional<IntervalOrExact>& State::get_delta_y_rear() const
{
    return delta_y_rear;
}

const std::optional<IntervalOrExact>& State::get_curvature() const
{
    return curvature;
}

const std::optional<IntervalOrExact>& State::get_curvature_change() const
{
    return curvature_change;
}

const std::optional<IntervalOrExact>& State::get_jerk() const
{
    return jerk;
}

const std::optional<IntervalOrExact>& State::get_jounce() const
{
    return jounce;
}
