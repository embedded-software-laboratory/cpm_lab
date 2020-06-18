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

#include "commonroad_classes/states/GoalState.hpp"

GoalState::GoalState(const xmlpp::Node* node)
{
    //2018 and 2020 specs are the same
    //Check if node is of type goal state
    assert(node->get_name() == "goalState");

    try
    {
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
            std::stringstream error_msg_stream;
            error_msg_stream << "No time node in GoalState (required by specification) - line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate GoalState:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    
    

    //Test output
    std::cout << "GoalState: " << std::endl;
    std::cout << "\tPosition exists: " << position.has_value() << std::endl;
    std::cout << "\tVelocity exists: " << velocity.has_value() << std::endl;
    std::cout << "\tOrientation exists: " << orientation.has_value() << std::endl;
    std::cout << "\tTime exists: " << time.has_value() << std::endl;
}

void GoalState::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (position.has_value())
    {
        position->transform_coordinate_system(scale, translate_x, translate_y);
    }

    if (scale > 0)
    {
        transform_scale *= scale;
    }
}

void GoalState::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    //Draw goal position
    if(position.has_value())
    {
        position->draw(ctx, scale, 0, 0, 0, local_orientation);
    }

    //Rotation is an interval - draw position for every possible orientation middle value
    if(orientation.has_value())
    {
        for (auto& middle : orientation->get_interval_avg())
        {
            ctx->save();
            ctx->set_source_rgb(1.0, 0.0, 0.0);

            if(position.has_value())
            {
                //Try to draw in the middle of the shape of the goal
                position->transform_context(ctx, scale);
            }
            ctx->rotate(middle + local_orientation);

            //Draw arrow
            double arrow_scale = scale * transform_scale; //To quickly change the scale to your liking
            draw_arrow(ctx, 0.0, 0.0, 1.0 * arrow_scale, 0.0, scale * transform_scale);
            
            ctx->restore();
        }
    }

    //TODO: Draw time, velocity?
    //Also TODO: Test output for other state classes

    ctx->restore();
}

void GoalState::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs)
{
    if(position.has_value())
    {
        position->set_lanelet_ref_draw_function(_draw_lanelet_refs);
    }
}