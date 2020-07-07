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

#include "commonroad_classes/geometry/Point.hpp"

Point::Point(const xmlpp::Node* node) 
{
    //Check if node is of type point
    assert(node->get_name() == "point" || node->get_name() == "center");

    try
    {
        //2018 and 2020
        x = xml_translation::get_child_child_double(node, "x", true).value(); //Must exist, error thrown if not, so we can use .value() without checking for its existence
        y = xml_translation::get_child_child_double(node, "y", true).value();

        //2020
        z = xml_translation::get_child_child_double(node, "z", false);
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Point:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    //Test output
    std::cout << "New point created: " << "(" << x << ", " << y << ", " << z.value_or(0) << ")" << std::endl;
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
Point::Point(int irrelevant_int)
{
    //This is probably the default value
    //We use this constructor because we do not want a default constructor to exist, but the parameter is actually pointless
    //If a point value is not given for a datatype where a position is necessary for its interpretation, we interpret this as a sign to use a 'default position' instead
    x = 0.0;
    y = 0.0;
}
#pragma GCC diagnostic pop

Point::Point(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = std::optional<double>(_z);
}

void Point::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (scale > 0)
    {
        x *= scale;
        y *= scale;
        
        if (z.has_value())
        {
            double new_z_value = z.value() * scale;
            z = std::optional<double>(new_z_value);
        }
    }

    x += translate_x;
    y += translate_y;
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void Point::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    //Current state: Leave out z-value
    //Idea for z-value: Add shade, or change saturation based on current cairo value, or put small number for height into point
    ctx->save();

    //Perform required translation + rotation
    //Local rotation does not really make sense here and is thus ignored (rotating a point in its own coordinate system is pointless)
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.03);
    ctx->set_line_cap(Cairo::LINE_CAP_ROUND);
    ctx->move_to(x * scale, y * scale);
    ctx->line_to(x * scale, y * scale);
    ctx->stroke();
    ctx->restore();
}
#pragma GCC diagnostic pop

CommonroadDDSPoint Point::to_dds_msg()
{
    CommonroadDDSPoint point;

    point.x(x);
    point.y(y);

    return point;
}

double Point::get_x()
{
    return x;
}

double Point::get_y()
{
    return y;
}

const std::optional<double>& Point::get_z() const
{
    return z;
}