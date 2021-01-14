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

#include "VehicleTrajectoryPlanningState.hpp" //sw folder central routing

#include "lane_graph_tools.hpp" //sw folder decentral routing


VehicleTrajectoryPlanningState::VehicleTrajectoryPlanningState(
    uint8_t _vehicle_id,
    size_t _edge_index,
    size_t _edge_path_index)
:vehicle_id(_vehicle_id)
,current_edge_index(_edge_index)
,current_edge_path_index(_edge_path_index)
,current_route_edge_indices({_edge_index})
{   
    // Set seed for rand
    srand(time(NULL)*vehicle_id);

    //finds the next n indizes of the edges of the graph for the trajectory for each vehicle 
    extend_random_route(500);

    speed_profile[0] = 0;
    for (size_t i = 1; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        speed_profile[i] = fmin(speed_profile[i-1] + delta_v_step, max_speed);
    }
}

void VehicleTrajectoryPlanningState::invariant()
{
    assert(current_route_edge_indices.size() >= 1);
    assert(current_route_edge_indices[0] == current_edge_index);
    assert(current_edge_index < laneGraphTools.n_edges);
    assert(current_edge_path_index < laneGraphTools.n_edge_path_nodes);
    assert(delta_s_path_node_offset >= 0);
}

void VehicleTrajectoryPlanningState::apply_timestep(uint64_t dt_nanos)
{
    uint64_t n_steps = dt_nanos / dt_speed_profile_nanos;
    assert(n_steps * dt_speed_profile_nanos == dt_nanos); // major timestep is multiple of minor timestep
    assert(n_steps * 2 < N_STEPS_SPEED_PROFILE);
    assert(n_steps >= 1);

    // calculate driving distance of this time step
    for (size_t i = 0; i < n_steps; ++i)
    {
        delta_s_path_node_offset += speed_profile[i] * dt_speed_profile;
    }

    // move our position in the lane graph by delta s
    laneGraphTools.move_along_route
    (
        current_route_edge_indices, 
        current_edge_index, 
        current_edge_path_index, 
        delta_s_path_node_offset
    );

    // delete old route edge(s)
    while( !current_route_edge_indices.empty()
        && current_route_edge_indices.at(0) != current_edge_index)
    {
        current_route_edge_indices.erase(current_route_edge_indices.begin());
    }

    extend_random_route(500);

    // shift and extend speed profile
    for (int i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        if(i + n_steps < N_STEPS_SPEED_PROFILE)
        {
            // Shift the current speed profile by n_steps if possible
            speed_profile[i] = speed_profile[i + n_steps];
        }
        else
        {
            // else, accelerate up to max_speed
            speed_profile[i] = fmin(speed_profile[i-1] + delta_v_step, max_speed);
        }
    }

    invariant();

    t_elapsed += dt_nanos;
}

void VehicleTrajectoryPlanningState::extend_random_route(size_t n)
{
    invariant();

    while(current_route_edge_indices.size() < n)
    {   //find next indizes for edges
        auto next_edges = laneGraphTools.find_subsequent_edges(current_route_edge_indices.back());
        assert(next_edges.size() > 0);
        current_route_edge_indices.push_back(next_edges.at(rand() % next_edges.size()));
    }
}

TrajectoryPoint VehicleTrajectoryPlanningState::get_trajectory_point()
{
    TrajectoryPoint trajectory_point;
    trajectory_point.t().nanoseconds(t_elapsed);
    trajectory_point.px(laneGraphTools.edges_x.at(current_edge_index).at(current_edge_path_index));
    trajectory_point.py(laneGraphTools.edges_y.at(current_edge_index).at(current_edge_path_index));
    trajectory_point.vx(laneGraphTools.edges_cos.at(current_edge_index).at(current_edge_path_index) * speed_profile[0]);
    trajectory_point.vy(laneGraphTools.edges_sin.at(current_edge_index).at(current_edge_path_index) * speed_profile[0]);
    return trajectory_point;
}



// Change the own speed profile so as not to collide with the other_vehicles.
// other_vehicles may only contain vehicles with a higher priority
bool VehicleTrajectoryPlanningState::avoid_collisions(
    std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles
)
{
    // TODO termination condition
    // for now: if this gets stuck the vehicles will stop, because they wont get a new command
    while(1)
    {
        vector<std::pair<size_t, size_t>> self_path = get_planned_path();

        // An index exceeding N_STEPS_SPEED_PROFILE indicates that there is no collision
        size_t earliest_collision__speed_profile_index = 1<<30;
        int colliding_vehicle_id = 0;

        // Find the earliest collision
        for( auto iter = other_vehicles.begin(); iter != other_vehicles.end(); ++iter ){
            std::map<size_t, std::pair<size_t, size_t>> other_path = iter->second;

            for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
            {
                // It's possible that we don't have data for this index
                // We will skip it, which is unsafe, but our only option right now
                if( other_path.count(i) == 0 ) { continue; }

                if(laneGraphTools.edge_path_collisions
                    [self_path[i].first]
                    [self_path[i].second]
                    [other_path[i].first]
                    [other_path[i].second])
                {
                    // collision detected
                    if(i < earliest_collision__speed_profile_index)
                    {
                        earliest_collision__speed_profile_index = i;
                        colliding_vehicle_id = iter->first;
                    }
                    // earliest collision for this vehicle found, stop
                    break;
                }
            }
        }

        // stop if there is no collision
        if(earliest_collision__speed_profile_index >= N_STEPS_SPEED_PROFILE) {
            return true;
        }


        cpm::Logging::Instance().write(2,
                "Detected potential collision; avoiding");

        // fix speed profile to avoid collision,
        // beginning from 10 steps before the collision
        // TODO: Change magic numbers to named parameters/constants
        int idx_speed_reduction = earliest_collision__speed_profile_index - 10;

        // If we already are almost at min_speed at the planned index for
        // speed reduction we cannot slow down much more,
        // so we try reducing the speed 10 steps earlier
        while(idx_speed_reduction > 0
            && speed_profile[idx_speed_reduction] < min_speed + 0.05)
        {
            idx_speed_reduction -= 10;
        }

        const double reduced_speed = fmax(speed_profile[idx_speed_reduction] - 0.3, min_speed);

        std::cout << 
            "Collision detected t:" << earliest_collision__speed_profile_index << 
            "  self id: " << int(vehicle_id) << 
            "  other id: " << colliding_vehicle_id << 
            "  Speed reduction,  t: " << idx_speed_reduction  << 
            "  spd: " << reduced_speed << std::endl;

        if(idx_speed_reduction < 15)
        {
            cpm::Logging::Instance().write(
                1,
                "Collision unavoidable between %d and %d.",
                int(vehicle_id), colliding_vehicle_id
            );
            std::cout << "Collision unavoidable" << std::endl;

            for(double spd:speed_profile)
            {
                std::cout << spd << ", ";
            }
            std::cout << std::endl;

            return false;
        }

        set_speed(idx_speed_reduction, reduced_speed);
    }

    return true;
}


void VehicleTrajectoryPlanningState::set_speed(int idx_speed_reduction, double speed_value)
{

    assert(idx_speed_reduction >= 0);
    assert(idx_speed_reduction < N_STEPS_SPEED_PROFILE);

    speed_profile[idx_speed_reduction] = speed_value;

    for (int i = 1; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        int i_forward = idx_speed_reduction + i;
        int i_reverse = idx_speed_reduction - i;

        if(i_forward < N_STEPS_SPEED_PROFILE)
        {
            speed_profile[i_forward] = fmin(speed_value + i * delta_v_step, speed_profile[i_forward]);
        }

        if(i_reverse >= 0)
        {
            speed_profile[i_reverse] = fmin(speed_value + i * delta_v_step, speed_profile[i_reverse]);
        }
    }
}

vector<std::pair<size_t, size_t>> VehicleTrajectoryPlanningState::get_planned_path()
{
    vector<std::pair<size_t, size_t>> result;
    
    double delta_s = delta_s_path_node_offset;
    for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        delta_s += speed_profile[i] * dt_speed_profile;
        double delta_s_copy = delta_s;

        size_t future_edge_index = current_edge_index;
        size_t future_edge_path_index = current_edge_path_index;

        laneGraphTools.move_along_route
        (
            current_route_edge_indices, 
            future_edge_index, 
            future_edge_path_index, 
            delta_s_copy
        );

        result.push_back(std::make_pair(future_edge_index, future_edge_path_index));
    }

    return result;
}

/*
 * Write our planned path into a LaneGraphTrajectory object
 */
void VehicleTrajectoryPlanningState::get_lane_graph_positions(
        LaneGraphTrajectory *lane_graph_trajectory)
{

    lane_graph_trajectory->vehicle_id(vehicle_id);
    
    std::vector<LaneGraphPosition> lane_graph_positions;

    for(std::pair<size_t, size_t> path_point:get_planned_path())
    {
        LaneGraphPosition lane_graph_pos;
        lane_graph_pos.edge_index(path_point.first);
        lane_graph_pos.edge_path_index(path_point.second);

        lane_graph_positions.push_back(lane_graph_pos);
    }

    lane_graph_trajectory->lane_graph_positions(lane_graph_positions);
}

void VehicleTrajectoryPlanningState::debug_writeOutOwnTrajectory() {
    std::cout << "Vehicle " << static_cast<uint32_t>(vehicle_id) << std::endl;
    int index = 0;
    for(auto point : get_planned_path()) {
        std::cout << index << ", "; 
        std::cout << point.first << ", "; 
        std::cout << point.second << std::endl; 
        index++;
    }
}
