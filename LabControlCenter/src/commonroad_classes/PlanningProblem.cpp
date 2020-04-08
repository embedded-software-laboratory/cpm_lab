#include "PlanningProblem.hpp"

PlanningProblem::PlanningProblem(const xmlpp::Node* node)
{
    //TODO: Assert node "type"

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
            goal_states.push_back(GoalState(child));
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

    //Require at least one goal state for each planning problem element (according to specs)
    for (const auto planning_problem : planning_problems)
    {
        assert(planning_problem.goal_states.size() > 0);
    }

    //Test output
    std::cout << "Translated Planning Problems: " << planning_problems.size() << std::endl;
}