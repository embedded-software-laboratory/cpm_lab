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

#include "commonroad_classes/StaticObstacle.hpp"

StaticObstacle::StaticObstacle(
    const xmlpp::Node* node,
    std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs,
    std::function<std::pair<double, double> (int)> _get_lanelet_center
    )
{
    //Warn in case node does not have static obstacle role
    //Check if node is of type staticObstacle
    assert(node->get_name() == "staticObstacle" || node->get_name() == "obstacle");
    if (node->get_name() == "obstacle") //2018 specs
    {
        std::string role_text = xml_translation::get_child_child_text(node, "role", true).value(); //Must exist (2018 specs), else error is thrown
        assert(role_text.compare("static") == 0);
    }
    
    commonroad_line = node->get_line();

    try
    {
        obstacle_type_text = xml_translation::get_child_child_text(node, "type", true).value(); //Must exist, error thrown anyway, so we can use .value() here
        if (obstacle_type_text.compare("unknown") == 0)
        {
            type = ObstacleTypeStatic::Unknown;
        }
        else if (obstacle_type_text.compare("parkedVehicle") == 0)
        {
            type = ObstacleTypeStatic::ParkedVehicle;
        }
        else if (obstacle_type_text.compare("constructionZone") == 0)
        {
            type = ObstacleTypeStatic::ConstructionZone;
        }
        else if (obstacle_type_text.compare("roadBoundary") == 0)
        {
            type = ObstacleTypeStatic::RoadBoundary;
        }
        else if (obstacle_type_text.compare("car") == 0 || 
            obstacle_type_text.compare("truck") == 0 || 
            obstacle_type_text.compare("bus") == 0 || 
            obstacle_type_text.compare("motorcycle") == 0 || 
            obstacle_type_text.compare("bicycle") == 0 || 
            obstacle_type_text.compare("pedestrian") == 0 || 
            obstacle_type_text.compare("priorityVehicle") == 0 || 
            obstacle_type_text.compare("train") == 0)
        {
            //Behavior for dynamic types, which should not be used here
            std::stringstream error_msg_stream;
            error_msg_stream << "Node element not conformant to specs - usage of dynamic type for static object (obstacleType), line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        else
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Node element not conformant to specs (static obstacle, obstacleType), line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        
        const auto shape_node = xml_translation::get_child_if_exists(node, "shape", true); //Must exist
        if (shape_node)
        {
            shape = std::optional<Shape>{std::in_place, shape_node};
        }

        const auto state_node = xml_translation::get_child_if_exists(node, "initialState", true); //Must exist
        if (state_node)
        {
            initial_state = std::optional<State>{std::in_place, state_node};
        }

        const auto trajectory_node = xml_translation::get_child_if_exists(node, "trajectory", false); //Must not exist
        const auto occupancy_node = xml_translation::get_child_if_exists(node, "occupancySet", false); //Must not exist
        if (trajectory_node || occupancy_node)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Trajectory / occupancy defined for static object (not allowed), line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate StaticObstacle:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    //Set lanelet_ref functions
    if(initial_state.has_value())
    {
        initial_state->set_lanelet_ref_draw_function(_draw_lanelet_refs);
        //initial_state->set_lanelet_get_center_function(_get_lanelet_center);
    }
    
}

/******************************Interface functions***********************************/

void StaticObstacle::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    if (scale > 0)
    {
        transform_scale *= scale;
    }
    
    if (shape.has_value())
    {
        shape->transform_coordinate_system(scale, 0.0, 0.0); //Shape does not need to be modified as well, because we already transform state/occupancy and initial state position values
    }

    if (initial_state.has_value())
    {
        initial_state->transform_coordinate_system(scale, translate_x, translate_y);
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void StaticObstacle::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    // ctx->save();

    // //Perform required translation + rotation
    // ctx->translate(global_translate_x, global_translate_y);
    // ctx->rotate(global_orientation);

    // //Different color / sticker / ... based on type
    // switch(type)
    // {
    //     case ObstacleTypeStatic::Unknown:
    //         ctx->set_source_rgb(0.1,0.1,0.1);
    //         break;
    //     case ObstacleTypeStatic::ParkedVehicle:
    //         ctx->set_source_rgb(0,0.1,0.5);
    //         break;
    //     case ObstacleTypeStatic::ConstructionZone:
    //         ctx->set_source_rgb(0.8,0,0.2);
    //         break;
    //     case ObstacleTypeStatic::RoadBoundary:
    //         ctx->set_source_rgb(0.5,0.5,0.5);
    //         break;
    // }

    // if (initial_state.has_value())
    // {
    //     initial_state->transform_context(ctx, scale);

    //     if (shape.has_value())
    //     {
    //         shape->draw(ctx, scale, 0, 0, 0, local_orientation);

    //         //Draw text on shape position
    //         //Set text position
    //         auto shape_center = shape->get_center();
    //         ctx->translate(shape_center.first, shape_center.second);
    //         ctx->rotate(local_orientation);

    //         //Set font, flip bc of chosen coordinate system
    //         ctx->select_font_face("sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_BOLD);
    //         Cairo::Matrix font_matrix(0.4 * scale * transform_scale, 0.0, 0.0, -0.4 * scale * transform_scale, 0.0, 0.0);
    //         ctx->set_font_matrix(font_matrix);

    //         //Calculate text width to center the text on the obstacle's center
    //         Cairo::TextExtents text_extents;
    //         ctx->get_text_extents(obstacle_type_text, text_extents);
    //         ctx->translate(-text_extents.width / 2.0, - text_extents.height / 2.0);

    //         //Draw text
    //         ctx->text_path(obstacle_type_text);
    //         ctx->set_source_rgb(1.0, 1.0, 1.0);
    //         ctx->fill_preserve();
    //         ctx->set_source_rgb(0.0, 0.0, 0.0);
    //         ctx->set_line_width(0.002 * scale);
    //         ctx->stroke();
    //     }
    //     else
    //     {
    //         std::stringstream error_stream;
    //         error_stream << "Cannot draw empty shape in static obstacle from line " << commonroad_line;
    //         LCCErrorLogger::Instance().log_error(error_stream.str());
    //     }
    // }
    // else
    // {
    //     std::stringstream error_stream;
    //     error_stream << "Cannot draw static obstacle from line " << commonroad_line << ", initial state is missing";
    //     LCCErrorLogger::Instance().log_error(error_stream.str());
    // }

    // ctx->restore();
}
#pragma GCC diagnostic pop

ObstacleSimulationData StaticObstacle::get_obstacle_simulation_data()
{
    ObstacleSimulationData simulation_data;

    //Add initial point
    ObstacleSimulationSegment initial_point;

    assert(initial_state.has_value()); //Must exist anyway at this point 
    //Required data must exist, this function may throw an error otherwise
    if (! initial_state->get_position().position_is_lanelet_ref())
    {
        initial_point.position = std::optional<std::pair<double, double>>(initial_state->get_position().get_center());
    }
    else
    {
        initial_point.lanelet_ref = initial_state->get_position().get_lanelet_ref();
    }
    
    initial_point.is_exact = initial_state->get_position().is_exact();
    initial_point.time = std::optional<IntervalOrExact>(initial_state->get_time());
    initial_point.orientation = initial_state->get_orientation_mean();
    //Optional data
    initial_point.velocity = initial_state->get_velocity();
    if (shape.has_value())
    {
        initial_point.shape = shape->to_dds_msg();
    }

    simulation_data.trajectory.push_back(initial_point);

    //Translate type to DDS type
    switch(type)
    {
        case ObstacleTypeStatic::ConstructionZone:
            simulation_data.obstacle_type = ObstacleType::ConstructionZone;
            break;
        case ObstacleTypeStatic::ParkedVehicle:
            simulation_data.obstacle_type = ObstacleType::ParkedVehicle;
            break;
        case ObstacleTypeStatic::RoadBoundary:
            simulation_data.obstacle_type = ObstacleType::RoadBoundary;
            break;
        case ObstacleTypeStatic::Unknown:
            simulation_data.obstacle_type = ObstacleType::Unknown;
            break;
    }
    
    return simulation_data;
}

ObstacleTypeStatic StaticObstacle::get_type()
{
    return type;
}

std::string StaticObstacle::get_obstacle_type_text()
{
    return obstacle_type_text;
}

const std::optional<Shape>& StaticObstacle::get_shape() const
{
    return shape;
}

const std::optional<State>& StaticObstacle::get_initial_state() const
{
    return initial_state;
}
