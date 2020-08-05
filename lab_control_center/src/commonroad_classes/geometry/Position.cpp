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

#include "commonroad_classes/geometry/Position.hpp"

Position::Position(const xmlpp::Node* node)
{
    //Check if node is of type position
    assert(node->get_name() == "position");

    try
    {
        //2018 and 2020
        const auto point_node = xml_translation::get_child_if_exists(node, "point", false); //not mandatory
        if (point_node)
        {
            point = std::optional<Point>(std::in_place, point_node);
        }

        commonroad_line = node->get_line();
        
        //Optional parts (all unbounded -> lists)
        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                circles.push_back(Circle(child));
            },
            "circle"
        );

        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                lanelet_refs.push_back(xml_translation::get_attribute_int(child, "ref", true).value()); //As mentioned in other classes: Value must exist, else error is thrown, so .value() can be used safely here
            },
            "lanelet"
        );

        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                polygons.push_back(Polygon(child));
            },
            "polygon"
        );

        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                rectangles.push_back(Rectangle(child));
            },
            "rectangle"
        );
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Position:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    //Test output
    std::cout << "Position:" << std::endl;
    std::cout << "\tPoint exists: " << point.has_value() << std::endl;
    std::cout << "\tCircle size: " << circles.size() << std::endl;
    std::cout << "\tLanelet ref size: " << lanelet_refs.size() << std::endl;
    std::cout << "\tPolygon size: " << polygons.size() << std::endl;
    std::cout << "\tRectangle size: " << rectangles.size() << std::endl;
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
Position::Position(int irrelevant_int)
{
    //If a position value is not given for a datatype where a position is necessary for its interpretation, we interpret this as a sign to use a 'default position' instead
    //We use this constructor because we do not want a default constructor to exist, but the parameter is actually pointless
    
    //As mentioned in the documentation, the default position is probably the origin
    point = std::optional<Point>(std::in_place, -1);
}
#pragma GCC diagnostic pop

void Position::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs)
{
    draw_lanelet_refs = _draw_lanelet_refs;
}

void Position::set_lanelet_get_center_function(std::function<std::pair<double, double> (int)> _get_lanelet_center)
{
    get_lanelet_center = _get_lanelet_center;
}

void Position::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (point.has_value())
    {
        point->transform_coordinate_system(scale, translate_x, translate_y);
    }

    for (auto& circle : circles)
    {
        circle.transform_coordinate_system(scale, translate_x, translate_y);
    }

    for (auto& polygon : polygons)
    {
        polygon.transform_coordinate_system(scale, translate_x, translate_y);
    }

    for (auto& rectangle : rectangles)
    {
        rectangle.transform_coordinate_system(scale, translate_x, translate_y);
    }

    if (scale > 0)
    {
        transform_scale *= scale;
    }
}

void Position::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);
    
    //Rotate, if necessary
    if(point.has_value())
    {
        point->draw(ctx, scale);

        //Draw circle around point for better visibility
        double radius = 0.75 * scale * transform_scale;
        ctx->set_line_width(0.005);
        ctx->arc(point->get_x() * scale, point->get_y() * scale, radius * scale, 0.0, 2 * M_PI);
        ctx->stroke();
    }
    else
    {
        //TODO: Rotation of combined forms is more complex than just rotation of the parts
        // for (auto circle : circles)
        // {
        //     circle.draw(ctx, scale, 0, 0, 0, local_orientation);
        // }
        // for (auto polygon : polygons)
        // {
        //     polygon.draw(ctx, scale, 0, 0, 0, local_orientation);
        // }
        // for (auto rectangle : rectangles)
        // {
        //     rectangle.draw(ctx, scale, 0, 0, 0, local_orientation);
        // }

        // if (lanelet_refs.size() > 0)
        // {
        //     if (draw_lanelet_refs)
        //     {
        //         for (auto lanelet_ref : lanelet_refs)
        //         {
        //             draw_lanelet_refs(lanelet_ref, ctx, scale, 0, 0, 0);
        //         }
        //     }
        //     else
        //     {
        //         std::cerr << "TODO: Better warning // Cannot draw using lanelet references - no lanelet ref draw function was set" << std::endl;
        //     }
        // }

        // if (circles.size() + polygons.size() + rectangles.size() > 1)
        // {
        //     std::cerr << "TODO: Better warning // Local rotation of position made of more than one form currently not supported" << std::endl;
        //     //TOOD: Implementation idea: Transform to center of the form, then rotate, then draw_relative_to(center_x, center_y) for the forms
        // }

        //Alternative rotation implementation: Rotate around overall center of the shape, then translate back to original coordinate system (but rotated), then draw
        ctx->save();

        auto center = get_center();
        ctx->translate(center.first * scale, center.second * scale);
        ctx->rotate(local_orientation);
        ctx->translate(-1.0 * center.first * scale, -1.0 * center.second * scale);

        for (auto circle : circles)
        {
            circle.draw(ctx, scale, 0, 0, 0, 0);
        }
        for (auto polygon : polygons)
        {
            polygon.draw(ctx, scale, 0, 0, 0, 0);
        }
        for (auto rectangle : rectangles)
        {
            rectangle.draw(ctx, scale, 0, 0, 0, 0);
        }

        if (lanelet_refs.size() > 0)
        {
            if (draw_lanelet_refs)
            {
                for (auto lanelet_ref : lanelet_refs)
                {
                    draw_lanelet_refs(lanelet_ref, ctx, scale, 0, 0, 0);
                }
            }
            else
            {
                LCCErrorLogger::Instance().log_error("Cannot draw without lanelet ref function in Position, set function callback beforehand!");
            }
        }

        ctx->restore();
    }

    ctx->restore();
}

void Position::transform_context(const DrawingContext& ctx, double scale)
{
    //Rotate, if necessary
    if(point.has_value())
    {
        ctx->translate(point->get_x() * scale, point->get_y() * scale);        
    }
    else
    {
        //TODO: Is the center the best point to choose?
        auto center = get_center();
        ctx->translate(center.first * scale, center.second * scale);

        if (circles.size() == 0 && polygons.size() == 0 && rectangles.size() == 0 && lanelet_refs.size() == 0)
        {
            std::stringstream error_stream;
            error_stream << "Cannot transform context in Position when position is empty, from line " << commonroad_line;
            LCCErrorLogger::Instance().log_error(error_stream.str());
        }
    }
}

std::pair<double, double> Position::get_center()
{
    //Return either exact point value or center value of inexact position
    if (point.has_value())
    {
        return std::pair<double, double>(point->get_x(), point->get_y());
    }

    double x, y = 0.0;
    double center_count = 1.0;

    for (auto circle : circles)
    {
        auto center = circle.get_center();
        x += (center.first - x) / center_count;
        y += (center.second - y) / center_count;
        ++center_count;
    }

    for (auto polygon : polygons)
    {
        auto center = polygon.get_center();
        x += (center.first - x) / center_count;
        y += (center.second - y) / center_count;
        ++center_count;
    }

    for (auto rectangle : rectangles)
    {
        auto center = rectangle.get_center();
        x += (center.first - x) / center_count;
        y += (center.second - y) / center_count;
        ++center_count;
    }

    if (get_lanelet_center)
    {
        for (auto lanelet_ref : lanelet_refs)
        {
            auto center = get_lanelet_center(lanelet_ref);
            x += (center.first - x) / center_count;
            y += (center.second - y) / center_count;
            ++center_count;
        }
    }
    else if (lanelet_refs.size() > 0)
    {
        LCCErrorLogger::Instance().log_error("Cannot compute center properly without lanelet center function in Position, set function callback beforehand!");
    }

    return std::pair<double, double>(x, y);
}

std::optional<int> Position::get_lanelet_ref()
{
    if (lanelet_refs.size() > 1)
    {
        std::stringstream error_stream;
        error_stream << "In Position: Cannot handle more than one lanelet ref, from line " << commonroad_line;
        LCCErrorLogger::Instance().log_error(error_stream.str());

        return std::optional<int>(lanelet_refs.at(0));
    }
    else if (lanelet_refs.size() == 1)
    {
        return std::optional<int>(lanelet_refs.at(0));
    }
    else 
    {
        return std::nullopt;
    }
}

bool Position::is_exact()
{
    if (point.has_value())
        return true;

    return false;
}

bool Position::position_is_lanelet_ref()
{
    return (lanelet_refs.size() > 0) && !(point.has_value()) && (circles.size() == 0) && (polygons.size() == 0) && (rectangles.size() == 0);
}

const std::optional<Point>& Position::get_point() const
{
    return point;
}

const std::vector<Circle>& Position::get_circles() const
{
    return circles;
}

const std::vector<int>& Position::get_lanelet_refs() const
{
    return lanelet_refs;
}

const std::vector<Polygon>& Position::get_polygons() const
{
    return polygons;
}

const std::vector<Rectangle>& Position::get_rectangles() const
{
    return rectangles;
}
