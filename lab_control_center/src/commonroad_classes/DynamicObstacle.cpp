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

#include "commonroad_classes/DynamicObstacle.hpp"

/**
 * \file DynamicObstacle.cpp
 * \ingroup lcc_commonroad
 */

DynamicObstacle::DynamicObstacle(
    const xmlpp::Node* node,
    std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs
    )
{
    //Check if node is of type dynamicObstacle
    assert(node->get_name() == "dynamicObstacle" || node->get_name() == "obstacle");
    if (node->get_name() == "obstacle") //2018 specs
    {
        std::string role_text = xml_translation::get_child_child_text(node, "role", true).value(); //Must exist (2018 specs), else error is thrown
        assert(role_text.compare("dynamic") == 0);
    }

    commonroad_line = node->get_line();

    try
    {
        obstacle_type_text = xml_translation::get_child_child_text(node, "type", true).value(); //Must exist, so an error is thrown anyway -> just use .value()
        if (obstacle_type_text.compare("unknown") == 0)
        {
            type = ObstacleTypeDynamic::Unknown;
        }
        else if (obstacle_type_text.compare("car") == 0)
        {
            type = ObstacleTypeDynamic::Car;
        }
        else if (obstacle_type_text.compare("truck") == 0)
        {
            type = ObstacleTypeDynamic::Truck;
        }
        else if (obstacle_type_text.compare("bus") == 0)
        {
            type = ObstacleTypeDynamic::Bus;
        }
        else if (obstacle_type_text.compare("motorcycle") == 0)
        {
            type = ObstacleTypeDynamic::Motorcycle;
        }
        else if (obstacle_type_text.compare("bicycle") == 0)
        {
            type = ObstacleTypeDynamic::Bicycle;
        }
        else if (obstacle_type_text.compare("pedestrian") == 0)
        {
            type = ObstacleTypeDynamic::Pedestrian;
        }
        else if (obstacle_type_text.compare("priorityVehicle") == 0)
        {
            type = ObstacleTypeDynamic::PriorityVehicle;
        }
        else if (obstacle_type_text.compare("train") == 0)
        {
            type = ObstacleTypeDynamic::Train;
        }
        else if (obstacle_type_text.compare("taxi") == 0)
        {
            type = ObstacleTypeDynamic::Taxi;
        }
        else if (obstacle_type_text.compare("parkedVehicle") == 0 || 
            obstacle_type_text.compare("constructionZone") == 0 || 
            obstacle_type_text.compare("roadBoundary") == 0)
        {
            //Behavior for dynamic types, which should not be used here
            throw SpecificationError("Node element not conformant to specs - usage of dynamic type for static object (obstacleType)");
        }
        else
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Node element not conformant to specs (obstacleType) in " << node->get_line();
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

        //Only one of the two must exist
        const auto trajectory_node = xml_translation::get_child_if_exists(node, "trajectory", false);
        const auto occupancy_node = xml_translation::get_child_if_exists(node, "occupancySet", false);
        const auto signal_node = xml_translation::get_child_if_exists(node, "signalSeries", false);
        if (! (trajectory_node || occupancy_node || signal_node))
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Trajectory / occupancy / signal series not defined for dynamic object (one must be defined) - line " << trajectory_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        if ((trajectory_node && occupancy_node) || (trajectory_node && signal_node) || (occupancy_node && signal_node))
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Trajectory / occupancy / signal series both / all three defined for dynamic object (only one must be defined) - line " << trajectory_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        //Translate trajectory, occupancy, signal series
        if (trajectory_node)
        {
            xml_translation::iterate_children(
                trajectory_node, 
                [&] (const xmlpp::Node* child) 
                {
                    trajectory.push_back(State(child));
                }, 
                "state"
            );
        }
        if (occupancy_node)
        {
            xml_translation::iterate_children(
                occupancy_node, 
                [&] (const xmlpp::Node* child) 
                {
                    occupancy_set.push_back(Occupancy(child));
                }, 
                "occupancy"
            );
        }
        //2020 only
        if (signal_node)
        {
            xml_translation::iterate_children(
                signal_node, 
                [&] (const xmlpp::Node* child) 
                {
                    signal_series.push_back(SignalState(child));
                }, 
                "signalState"
            );
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate DynamicObstacle:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    //Set lanelet ref functions for geometry
    if(initial_state.has_value())
    {
        initial_state->set_lanelet_ref_draw_function(_draw_lanelet_refs);
        //initial_state->set_lanelet_get_center_function(_get_lanelet_center);
    }
    for (auto& state : trajectory)
    {
        state.set_lanelet_ref_draw_function(_draw_lanelet_refs);
        //state.set_lanelet_get_center_function(_get_lanelet_center);
    }
    

    //Test output
    // std::cout << "Dynamic obstacle:" << std::endl;
    // std::cout << "\tTrajectory size: " << trajectory.size() << std::endl;
    // std::cout << "\tOccupancy size: " << occupancy_set.size() << std::endl;
    // std::cout << "\tSignal series size: " << signal_series.size() << std::endl;
    // std::cout << "\tObstacle type (text, before translation to enum): " << obstacle_type_text << std::endl;
    // std::cout << "\tInitial state exists (it should): " << initial_state.has_value() << std::endl;
    // std::cout << "\tShape exists (it should): " << shape.has_value() << std::endl;
} 

/******************************Interface functions***********************************/

void DynamicObstacle::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    if (scale > 0)
    {
        transform_scale *= scale;
    }
    
    if (shape.has_value())
    {
        shape->transform_coordinate_system(scale, angle, 0.0, 0.0); //Shape does not need to be modified as well, because we already transform state/occupancy and initial state position values
    }

    if (initial_state.has_value())
    {
        initial_state->transform_coordinate_system(scale, angle, translate_x, translate_y);
    }

    for (auto& state : trajectory)
    {
        state.transform_coordinate_system(scale, angle, translate_x, translate_y);
    }

    for (auto& occupancy : occupancy_set)
    {
        occupancy.transform_coordinate_system(scale, angle, translate_x, translate_y);
    }
}

void DynamicObstacle::transform_timing(double time_scale)
{
    if (initial_state.has_value())
    {
        initial_state->transform_timing(time_scale);
    }

    for (auto& state : trajectory)
    {
        state.transform_timing(time_scale);
    }
}

void DynamicObstacle::draw_shape_with_text(const DrawingContext& ctx, double scale, double local_orientation)
{
    if (shape.has_value())
    {
        shape->draw(ctx, scale, 0, 0, 0, local_orientation);

        //Draw text on shape position
        //Set text position
        draw_text(ctx, scale, local_orientation, shape->get_center());
    }
    else
    {
        std::stringstream error_stream;
        error_stream << "Cannot draw dynamic obstacle shape (empty), from line " << commonroad_line;
        LCCErrorLogger::Instance().log_error(error_stream.str());
    }
}

void DynamicObstacle::draw_text(const DrawingContext& ctx, double scale, double local_orientation, std::pair<double, double> center)
{
    ctx->translate(center.first, center.second);
    ctx->rotate(local_orientation);

    //Set font, flip bc of chosen coordinate system
    ctx->select_font_face("sans", Cairo::FONT_SLANT_NORMAL, Cairo::FONT_WEIGHT_BOLD);
    Cairo::Matrix font_matrix(0.4 * scale * transform_scale, 0.0, 0.0, -0.4 * scale * transform_scale, 0.0, 0.0);
    ctx->set_font_matrix(font_matrix);

    //Calculate text width to center the text on the obstacle's center
    Cairo::TextExtents text_extents;
    ctx->get_text_extents(obstacle_type_text, text_extents);
    ctx->translate(-text_extents.width / 2.0, - text_extents.height / 2.0);

    //Draw text
    ctx->text_path(obstacle_type_text);
    ctx->set_source_rgb(1.0, 1.0, 1.0);
    ctx->fill_preserve();
    ctx->set_source_rgb(0.0, 0.0, 0.0);
    ctx->set_line_width(0.002 * scale);
    ctx->stroke();
}

ObstacleSimulationData DynamicObstacle::get_obstacle_simulation_data()
{
    ObstacleSimulationData commonroad_trajectory;

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

    commonroad_trajectory.trajectory.push_back(initial_point);


    if (trajectory.size() > 0)
    {
        for (auto& point : trajectory)
        {
            ObstacleSimulationSegment trajectory_point;

            //Required data must exist, this function may throw an error otherwise
            if (! point.get_position().position_is_lanelet_ref())
            {
                trajectory_point.position = std::optional<std::pair<double, double>>(point.get_position().get_center());
            }
            else
            {
                trajectory_point.lanelet_ref = point.get_position().get_lanelet_ref();
            }
            
            trajectory_point.is_exact = point.get_position().is_exact();
            trajectory_point.time = std::optional<IntervalOrExact>(point.get_time());
            trajectory_point.orientation = point.get_orientation_mean();
            //Optional data
            trajectory_point.velocity = point.get_velocity();

            //Shape data (never changes for trajectory type)
            if (shape.has_value())
            {
                trajectory_point.shape = shape->to_dds_msg();
            }

            commonroad_trajectory.trajectory.push_back(trajectory_point);
        }
    }
    else if (occupancy_set.size() > 0)
    {
        for (auto& point : occupancy_set)
        {
            ObstacleSimulationSegment trajectory_point;

            //Either use occupancy or shape, if that exists
            auto occupancy_shape = point.get_shape();
            if (occupancy_shape.has_value())
            {
                trajectory_point.shape = occupancy_shape->to_dds_msg();   
            }
            else if (shape.has_value())
            {
                trajectory_point.shape = shape->to_dds_msg();
            }

            //Required data must exist, this function may throw an error otherwise - there is no lanelet ref in occupancy
            trajectory_point.time = std::optional<IntervalOrExact>(point.get_time());

            //Point values are given by the shape implicitly
            //trajectory_point.position = std::optional<std::pair<double, double>>(point.get_center());
            //trajectory_point.orientation = point.get_orientation(); Is already within shape
            trajectory_point.position = std::optional<std::pair<double, double>>(std::in_place, 0.0, 0.0);
            trajectory_point.orientation = std::optional<double>(0.0);
            //Velocity data does not exist in this case

            trajectory_point.is_exact = false; //Occupancy values are never exact, because they define an occupied area

            commonroad_trajectory.trajectory.push_back(trajectory_point);
        }
    }

    //Translate type to DDS type
    switch(type)
    {
        case ObstacleTypeDynamic::Bicycle:
            commonroad_trajectory.obstacle_type = ObstacleType::Bicycle;
            break;
        case ObstacleTypeDynamic::Bus:
            commonroad_trajectory.obstacle_type = ObstacleType::Bus;
            break;
        case ObstacleTypeDynamic::Car:
            commonroad_trajectory.obstacle_type = ObstacleType::Car;
            break;
        case ObstacleTypeDynamic::Motorcycle:
            commonroad_trajectory.obstacle_type = ObstacleType::Motorcycle;
            break;
        case ObstacleTypeDynamic::Pedestrian:
            commonroad_trajectory.obstacle_type = ObstacleType::Pedestrian;
            break;
        case ObstacleTypeDynamic::PriorityVehicle:
            commonroad_trajectory.obstacle_type = ObstacleType::PriorityVehicle;
            break;
        case ObstacleTypeDynamic::Taxi:
            commonroad_trajectory.obstacle_type = ObstacleType::Taxi;
            break;
        case ObstacleTypeDynamic::Train:
            commonroad_trajectory.obstacle_type = ObstacleType::Train;
            break;
        case ObstacleTypeDynamic::Truck:
            commonroad_trajectory.obstacle_type = ObstacleType::Truck;
            break;
        case ObstacleTypeDynamic::Unknown:
            commonroad_trajectory.obstacle_type = ObstacleType::Unknown;
            break;
        default:
            throw std::runtime_error("Translation to DDS type failed, unsupported type (-> programming error, add this type!)");
            break;
    }

    //Add class type
    commonroad_trajectory.obstacle_class = ObstacleClass::Dynamic;
    
    return commonroad_trajectory;
}

std::string DynamicObstacle::get_obstacle_type_text()
{
    return obstacle_type_text;
}

ObstacleTypeDynamic DynamicObstacle::get_type()
{
    return type;
}

const std::optional<Shape>& DynamicObstacle::get_shape() const
{
    return shape;
}

const std::optional<State>& DynamicObstacle::get_initial_state() const
{
    return initial_state;
}

const std::optional<SignalState>& DynamicObstacle::get_initial_signal_state() const
{
    return initial_signal_state;
}

const std::vector<State>& DynamicObstacle::get_trajectory() const
{
    return trajectory;
}

const std::vector<Occupancy>& DynamicObstacle::get_occupancy_set() const
{
    return occupancy_set;
}

const std::vector<SignalState>& DynamicObstacle::get_signal_series() const
{
    return signal_series;
}
