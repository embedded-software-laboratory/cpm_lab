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

#include "PlanningProblem.hpp"

/**
 * \file PlanningProblem.cpp
 * \ingroup lcc_commonroad
 */

PlanningProblem::PlanningProblem(
    const xmlpp::Node* node,
    std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs,
    std::function<std::pair<double, double> (int)> _get_lanelet_center,
    std::shared_ptr<CommonroadDrawConfiguration> _draw_configuration
    ) :
    draw_configuration(_draw_configuration)
{
    //Check if node is of type planningProblem
    assert(node->get_name() == "planningProblem");

    try
    {
        //Get the planning problem ID to tell it to the goal states (to be able to show the user the goal state ID in the UI)
        planning_problem_id = xml_translation::get_attribute_int(node, "id", true).value();

        //Due to ambiguities regarding the use of xs:sequence in the specs
        //compared to the implications made in the spec PDF, we make sure
        //that really only one entry for initial state exists
        //(interpretation of the .xsd file alone could be different)
        bool first_initial_state = true;
        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                if (!first_initial_state)
                {
                    std::stringstream error_msg_stream;
                    error_msg_stream << "Only one initial state allowed in a planning problem, line: " << node->get_line();
                    throw SpecificationError(error_msg_stream.str());
                }

                initial_state = std::optional<StateExact>(std::in_place, child);
                
                first_initial_state = false;
            },
            "initialState"
        );

        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                goal_states.push_back(GoalState(child, _draw_lanelet_refs, _get_lanelet_center, _draw_configuration));
            },
            "goalState"
        );
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate PlanningProblem:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

    //Require at least one goal state (according to specs)
    if(goal_states.size() == 0)
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Goal states missing in translated planning problem: " << node->get_name() << "; line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    //Set lanelet_ref function
    initial_state->set_lanelet_ref_draw_function(_draw_lanelet_refs);
    
    //Set unique goal ID
    size_t goal_pos = 0;
    for (auto& goal : goal_states)
    {
        //Create unique ID for each goal state (for identification in the table of goal state information)
        std::stringstream goal_id_stream;
        goal_id_stream << planning_problem_id << "." << goal_pos;
        goal.set_unique_id(goal_id_stream.str());

        ++goal_pos;
    }

    //Test output
    // std::cout << "Translated Planning Problems: " << planning_problems.size() << std::endl;
}

void PlanningProblem::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    if (initial_state.has_value())
    {
        initial_state->transform_coordinate_system(scale, angle, translate_x, translate_y);
    }

    for (auto& goal_state : goal_states)
    {
        goal_state.transform_coordinate_system(scale, angle, translate_x, translate_y);
    }
}

void PlanningProblem::transform_timing(double time_scale)
{
    if (initial_state.has_value())
    {
        initial_state->transform_timing(time_scale);
    }

    for (auto& goal_state : goal_states)
    {
        goal_state.transform_timing(time_scale);
    }
}

void PlanningProblem::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    assert(draw_configuration);

    ctx->save();

    //Perform required translation + rotation
    //Local orientation is irrelevant here
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    //Draw initial state
    ctx->set_source_rgb(0.0,0.5,0.05);
    initial_state->draw(ctx, scale, 0, 0, 0, local_orientation);

    //Draw initial state description (planning problem ID)
    if (draw_configuration->draw_init_state.load())
    {
        ctx->save();
        std::stringstream descr_stream;
        descr_stream << "ID (" << planning_problem_id << "): ";
        initial_state->transform_context(ctx, scale);

        //Draw set text. Re-scale text based on current zoom factor
        draw_text_centered(ctx, 0, 0, 0, 1200.0 / draw_configuration->zoom_factor.load(), descr_stream.str());
        ctx->restore();
    }

    //Draw goal state + description
    ctx->set_source_rgba(1.0,0.5,0.8, 0.3);
    for (auto goal : goal_states)
    {
        goal.draw(ctx, scale, 0, 0, 0, local_orientation);
    }

    ctx->restore();
}

const std::optional<StateExact>& PlanningProblem::get_initial_state() const
{
    return initial_state;
}

const std::vector<GoalState>& PlanningProblem::get_goal_states() const
{
    return goal_states;
}

std::vector<CommonroadDDSGoalState> PlanningProblem::get_dds_goal_states(double time_step_size)
{
    std::vector<CommonroadDDSGoalState> commonroad_goal_states;

    for (size_t goal_pos = 0; goal_pos < goal_states.size(); ++ goal_pos)
    { 
        auto dds_goal_state = goal_states.at(goal_pos).to_dds_msg(time_step_size);
        dds_goal_state.goal_state_pos(goal_pos);
        commonroad_goal_states.push_back(dds_goal_state);
    }

    return commonroad_goal_states;
}