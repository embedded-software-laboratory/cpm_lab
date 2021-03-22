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

#include "commonroad_classes/geometry/Rectangle.hpp"

/**
 * \file Rectangle.cpp
 * \ingroup lcc_commonroad
 */

Rectangle::Rectangle(const xmlpp::Node* node)
{
    //Check if node is of type rectangle
    assert(node->get_name() == "rectangle");

    try
    {
        length = xml_translation::get_child_child_double(node, "length", true).value(); //mandatory, so we can use .value() bc an error is thrown before anyway if it does not exist
        width = xml_translation::get_child_child_double(node, "width", true).value(); //mandatory, see above

        if (length < 0 || width < 0)
        {
            throw SpecificationError(std::string("Could not translate Rectangle - length or width is smaller than zero"));
        }

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

        orientation = xml_translation::get_child_child_double(node, "orientation", false);
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Rectangle:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

    //Test output
    // std::cout << "Rectangle:" << std::endl;
    // std::cout << "\tLenght, width: " << length << ", " << width << std::endl;
    // std::cout << "\tOrientation set: " << orientation.has_value() << std::endl;
    // std::cout << "\tCenter set: " << center.has_value() << std::endl;
}

void Rectangle::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    center->transform_coordinate_system(scale, angle, translate_x, translate_y);

    //Change rotation itself as well; rotation is counter-clockwise
    //TODO: Is it okay to just add to the rotation in local coordinates if the points have already been converted correctly?
    auto old_value = orientation.value_or(0.0);
    orientation = std::optional<double>(rotate_orientation_around_z(old_value, angle));
    

    if (scale > 0)
    {
        length *= scale;
        width *= scale;
    }
}

void Rectangle::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.005);

    //Translate to center of object
    ctx->translate(center->get_x() * scale, center->get_y() * scale);

    //Rotate, if necessary
    if (orientation.has_value())
    {
        ctx->rotate(orientation.value());
    }
    //Also perform desired local orientation change
    ctx->rotate(local_orientation);

    //Move to first corner from center
    ctx->move_to((- (length/2)) * scale, (- (width/2)) * scale);

    //Draw lines
    ctx->line_to((- (length/2)) * scale, (  (width/2)) * scale);
    ctx->line_to((  (length/2)) * scale, (  (width/2)) * scale);
    ctx->line_to((  (length/2)) * scale, (- (width/2)) * scale);
    ctx->line_to((- (length/2)) * scale, (- (width/2)) * scale);
    ctx->fill_preserve();
    ctx->stroke();

    ctx->restore();
}

//********************************************************************************************************************************************
//Getter
//********************************************************************************************************************************************

std::pair<double, double> Rectangle::get_center()
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

CommonroadDDSRectangle Rectangle::to_dds_msg()
{
    CommonroadDDSRectangle rectangle;

    rectangle.length(length);
    rectangle.width(width);

    if(center.has_value())
    {
        rectangle.center(center->to_dds_msg());
    }
    else
    {
        rectangle.center(Point(-1).to_dds_msg()); //Default position
    }
    

    rectangle.orientation(orientation.value_or(0)); //Orientation is 0 if not set

    return rectangle;
}

std::optional<double> Rectangle::get_orientation()
{
    return orientation;
}

const std::optional<Point>& Rectangle::get_center() const
{
    return center;
}

double Rectangle::get_length()
{
    return length;
}

double Rectangle::get_width()
{
    return width;
}
