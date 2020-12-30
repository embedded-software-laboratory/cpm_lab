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
    )
{
    //Check if node is of type planningProblem
    assert(node->get_name() == "planningProblem");

    try
    {
        //Initial state and goal state can be sequences, which are here translated separately and then put together
        //As there might be several goal states for one initial state, we save line numbers here as well
        std::vector<std::optional<StateExact>> initial_states;
        std::vector<int> initial_states_lines;
        std::vector<GoalState> goal_states;
        std::vector<int> goal_state_lines;

        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                initial_states.push_back(std::optional<StateExact>(std::in_place, child));
                initial_states_lines.push_back(child->get_line());
            },
            "initialState"
        );
        xml_translation::iterate_children(
            node,
            [&] (const xmlpp::Node* child)
            {
                goal_states.push_back(GoalState(child, _draw_lanelet_refs, _get_lanelet_center, _draw_configuration));
                goal_state_lines.push_back(child->get_line());
            },
            "goalState"
        );

        //Create PlanningProblem elements depending on the line values (use std bound functions for search etc!)
        size_t goals_index = 0; //Store up to which index additional_values have already been stored in previous elements
        for (size_t state_index = 0; state_index < initial_states.size(); ++state_index)
        {
            //Set up traffic post consisting of one ID and possibly several additional values
            PlanningProblemElement planning_problem;
            planning_problem.initial_state = initial_states.at(state_index);

            if (state_index == initial_states.size() - 1)
            {
                //Last element, take all remaining additional values
                if (goal_states.size() > goals_index)
                {
                    std::copy(goal_states.begin() + goals_index, goal_states.end(), std::back_inserter(planning_problem.goal_states));
                }
            }
            else
            {
                //Take all additional values up to upper bound (using the starting line of the next traffic post)
                int next_problem_line = goal_state_lines.at(state_index + 1);
                auto next_goal_it = std::upper_bound(goal_state_lines.begin(), goal_state_lines.end(), next_problem_line);

                if (next_goal_it != goal_state_lines.end())
                {
                    //There is an upper bound, copy up to next additional values of next element
                    auto next_goal_index = std::distance(goal_state_lines.begin(), next_goal_it);
                    std::copy(goal_states.begin() + goals_index, goal_states.begin() + next_goal_index, std::back_inserter(planning_problem.goal_states));

                    goals_index = next_goal_index;
                }
                else if (goal_states.size() > goals_index)
                {
                    //There is no upper bound, but there are still elements left - take all remaining values
                    std::copy(goal_states.begin() + goals_index, goal_states.end(), std::back_inserter(planning_problem.goal_states));

                    goals_index = goal_states.size();
                }
            }
            
            planning_problems.push_back(planning_problem);
        }
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
    

    //Require at least one goal state for each planning problem element (according to specs)
    for (const auto planning_problem : planning_problems)
    {
        if(planning_problem.goal_states.size() == 0)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Goal states missing in translated planning problem: " << node->get_name() << "; line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }

    //Set lanelet_ref functions
    for (auto& planning_prob : planning_problems)
    {
        planning_prob.initial_state->set_lanelet_ref_draw_function(_draw_lanelet_refs);
        //planning_prob.initial_state->set_lanelet_get_center_function(_get_lanelet_center);
    }

    //Test output
    // std::cout << "Translated Planning Problems: " << planning_problems.size() << std::endl;
}

void PlanningProblem::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    for (auto& planning_problem : planning_problems)
    {
        if (planning_problem.initial_state.has_value())
        {
            planning_problem.initial_state->transform_coordinate_system(scale, angle, translate_x, translate_y);
        }

        for (auto& goal_state : planning_problem.goal_states)
        {
            goal_state.transform_coordinate_system(scale, angle, translate_x, translate_y);
        }
    }
}

void PlanningProblem::transform_timing(double time_scale)
{
    for (auto& planning_problem : planning_problems)
    {
        if (planning_problem.initial_state.has_value())
        {
            planning_problem.initial_state->transform_timing(time_scale);
        }

        for (auto& goal_state : planning_problem.goal_states)
        {
            goal_state.transform_timing(time_scale);
        }
    }
}

void PlanningProblem::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    //Local orientation is irrelevant here
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    for (auto problem : planning_problems)
    {
        ctx->set_source_rgb(0.0,0.5,0.05);
        problem.initial_state->draw(ctx, scale, 0, 0, 0, local_orientation);

        ctx->set_source_rgba(1.0,0.5,0.8, 0.3);
        for (auto goal : problem.goal_states)
        {
            goal.draw(ctx, scale, 0, 0, 0, local_orientation);
        }
    }

    ctx->restore();
}

const std::vector<PlanningProblemElement>& PlanningProblem::get_planning_problems() const
{
    return planning_problems;
}

std::vector<CommonroadDDSGoalState> PlanningProblem::get_dds_goal_states(double time_step_size)
{
    std::vector<CommonroadDDSGoalState> goal_states;

    for (size_t planning_pos = 0; planning_pos < planning_problems.size(); ++planning_pos)
    {
        for (size_t goal_pos = 0; goal_pos < planning_problems.at(planning_pos).goal_states.size(); ++ goal_pos)
        { 
            auto dds_goal_state = planning_problems.at(planning_pos).goal_states.at(goal_pos).to_dds_msg(time_step_size);
            dds_goal_state.goal_state_pos(goal_pos);
            dds_goal_state.planning_problem_pos(planning_pos);
            goal_states.push_back(dds_goal_state);
        }
    }

    return goal_states;
}