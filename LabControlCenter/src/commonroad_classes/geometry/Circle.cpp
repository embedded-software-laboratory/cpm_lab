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

#include "commonroad_classes/geometry/Circle.hpp"

Circle::Circle(const xmlpp::Node* node)
{
    //Check if node is of type circle
    assert(node->get_name() == "circle");

    try
    {
        radius = xml_translation::get_child_child_double(node, "radius", true).value(); //mandatory, error thrown if nonexistant, so we can use .value() here

        //Get point value, which must not be specified
        const auto point_node = xml_translation::get_child_if_exists(node, "center", false);
        if (point_node)
        {
            center = std::optional<Point>{std::in_place, point_node};
        }
        else
        {
            //Use default-value constructor (parameter is irrelevant)
            center = std::optional<Point>{std::in_place, 0};
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Circle:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    //Test output
    std::cout << "Circle:" << std::endl;
    std::cout << "\tRadius: " << radius << std::endl;
    std::cout << "\tCenter set: " << center.has_value() << std::endl;
}

void Circle::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (scale > 0)
    {
        radius *= scale;
    }
    center->transform_coordinate_system(scale, translate_x, translate_y);
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void Circle::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    //Local rotation does not really make sense here and is thus ignored (rotating a circle in its own coordinate system is pointless)
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.005);

    //Move to center
    ctx->move_to(center->get_x() * scale, center->get_y() * scale);

    //Draw circle
    ctx->arc(center->get_x() * scale, center->get_y() * scale, radius * scale, 0.0, 2 * M_PI);
    ctx->stroke();

    ctx->restore();
}
#pragma GCC diagnostic pop

std::pair<double, double> Circle::get_center()
{
    if (center.has_value())
    {
        return std::pair<double, double>(center->get_x(), center->get_y());
    }
    else
    {
        //Return "default value", though this should never be reached if the constructor worked
        auto default_center = Point(-1);
        return std::pair<double, double>(default_center.get_x(), default_center.get_y());
    }
}

const std::optional<Point>& Circle::get_center() const
{
    return center;
}