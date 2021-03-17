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

#include "commonroad_classes/states/StateExact.hpp"

/**
 * \file StateExact.cpp
 * \ingroup lcc_commonroad
 */

StateExact::StateExact(const xmlpp::Node* node)
{
    //2018 and 2020 specs are the same
    //Check if node is of type stateExact
    assert(node->get_name() == "initialState");

    try
    {
        const auto position_node = xml_translation::get_child_if_exists(node, "position", true);
        if (position_node)
        {
            position = std::optional<Position>{std::in_place, position_node};

            if (! position->is_exact())
            {
                std::stringstream error_stream;
                error_stream << "Position must be exact, in line: ";
                error_stream << position_node->get_line();
                throw SpecificationError(error_stream.str());
            }
        }
        else
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "No position node in StateExact - is this an error or do you want to use the default value? - Line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());

            //Use default-value constructor (parameter is irrelevant)
            //position = std::optional<Position>{std::in_place, 0};
        }

        velocity = xml_translation::get_child_child_double_exact(node, "velocity", true).value(); //We can use .value() here, because if none exists an error is thrown beforehand
        orientation = xml_translation::get_child_child_double_exact(node, "orientation", true).value(); //We can use .value() here, because if none exists an error is thrown beforehand
        yaw_rate = xml_translation::get_child_child_double_exact(node, "yawRate", true).value(); //We can use .value() here, because if none exists an error is thrown beforehand
        slip_angle = xml_translation::get_child_child_double_exact(node, "slipAngle", true).value(); //We can use .value() here, because if none exists an error is thrown beforehand

        //Acceleration must not exist
        acceleration = xml_translation::get_child_child_double_exact(node, "acceleration", false);

        //Warn if time is not specified; must always be zero according to specs
        time = xml_translation::get_child_child_double_exact(node, "time", true).value(); //We can use .value() here, because if none exists an error is thrown beforehand
        if (time != 0.0)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Only a time value of exactly 0 is allowed here - line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate StateExact:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

    //Test output
    // std::cout << "StateExact: " << std::endl;
    // std::cout << "\tPosition exists: " << position.has_value() << std::endl;
    // std::cout << "\tVelocity: " << velocity << std::endl;
    // std::cout << "\tAcceleration defined: " << acceleration.has_value() << std::endl;
    // std::cout << "\tOrientation: " << orientation << std::endl;
    // std::cout << "\tYaw rate: " << yaw_rate << std::endl;
    // std::cout << "\tSlip angle: " << slip_angle << std::endl;
    // std::cout << "\tTime: " << time << std::endl;
}

void StateExact::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    if (position.has_value())
    {
        position->transform_coordinate_system(scale, angle, translate_x, translate_y);
    }

    //If all positional values are adjusted, the velocity must be adjusted as well
    velocity *= scale;
    if (acceleration.has_value())
    {
        auto new_acceleration = scale * acceleration.value();
        acceleration = std::optional<double>(new_acceleration);
    }

    if (scale > 0)
    {
        transform_scale *= scale;   
    }
}

void StateExact::transform_timing(double time_scale)
{
    if (time_scale > 0)
    {
        velocity *= time_scale;

        if (acceleration.has_value())
        {
            auto new_acceleration = time_scale * acceleration.value();
            acceleration = std::optional<double>(new_acceleration);
        }

        yaw_rate *= time_scale;
    }
}

void StateExact::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    position->draw(ctx, scale);

    //Draw arrow with correct position / rotation
    position->transform_context(ctx, scale);
    ctx->rotate(orientation + local_orientation);

    double arrow_scale = scale * transform_scale; //To quickly change the scale to your liking
    draw_arrow(ctx, 0.0, 0.0, 2.0 * arrow_scale, 0.0, 2.0 * arrow_scale);

    ctx->restore();
}

void StateExact::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs)
{
    if(position.has_value())
    {
        position->set_lanelet_ref_draw_function(_draw_lanelet_refs);
    }
}

void StateExact::transform_context(const DrawingContext& ctx, double scale)
{
    //Draw at the stored position (if possible), else somehow within the possible position
    position->transform_context(ctx, scale);
    
    //Rotate, if necessary
    ctx->rotate(orientation);
}

const std::optional<Position>& StateExact::get_position() const
{
    return position;
}

double StateExact::get_orientation()
{
    return orientation;
}

double StateExact::get_time()
{
    return time;
}

double StateExact::get_velocity()
{
    return velocity;
}

const std::optional<double> StateExact::get_acceleration() const
{
    return acceleration;
}

double StateExact::get_yaw_rate()
{
    return yaw_rate;
}

double StateExact::get_slip_angle()
{
    return slip_angle;
}
