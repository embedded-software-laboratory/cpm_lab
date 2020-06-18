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

#include "commonroad_classes/states/Occupancy.hpp"

Occupancy::Occupancy(const xmlpp::Node* node)
{
    //Check if node is of type occupancy
    assert(node->get_name() == "occupancy");

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
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Occupancy:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
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

//********************************************************************************************************************************************
//Getter
//********************************************************************************************************************************************

std::pair<double, double> Occupancy::get_center()
{
    if (!shape.has_value())
    {
        throw SpecificationError("Occupancy should have shape value, but does not");
    }
    return shape->get_center();
}

IntervalOrExact Occupancy::get_time()
{
    if (!time.has_value())
    {
        throw SpecificationError("Occupancy should have time value, but does not");
    }
    return time.value();
}

std::optional<double> Occupancy::get_orientation()
{
    if (!shape.has_value())
    {
        throw SpecificationError("Occupancy should have shape value, but does not");
    }
    return shape->get_orientation();
}