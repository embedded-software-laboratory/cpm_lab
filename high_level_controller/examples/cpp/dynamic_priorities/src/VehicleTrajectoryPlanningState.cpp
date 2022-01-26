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

#include "lane_graph_tools.hpp" //sw folder distributed routing
#include <string>
#include <iostream>
#include <math.h>       /* ceil */
#include "ovals_example.hpp" // oval example trajectories

/**
 * \file VehicleTrajectoryPlanningState.cpp
 * \ingroup distributed_routing
 */

VehicleTrajectoryPlanningState::VehicleTrajectoryPlanningState(
    uint8_t _vehicle_id,
    size_t _edge_index,
    size_t _edge_path_index,
    uint64_t dt_nanos,
    int _seed
)
:vehicle_id(_vehicle_id)
,current_edge_index(_edge_index)
,current_edge_path_index(_edge_path_index)
,current_route_edge_indices({_edge_index})
,dt_nanos(dt_nanos)
,n_steps(dt_nanos / dt_speed_profile_nanos)
{   

    assert(n_steps * dt_speed_profile_nanos == dt_nanos); // major timestep is multiple of minor timestep
    assert(n_steps * 2 < N_STEPS_SPEED_PROFILE);
    assert(n_steps >= 1);
    // Set seed for rand; for reproducibility; is used to generate paths; 
    random_gen.seed((uint8_t)_seed);

    //finds the next n indizes of the edges of the graph for the trajectory for each vehicle; important: done after seed initialisation
    extend_random_route(500);

    // stand still
    speed_profile.fill(0);
}

void VehicleTrajectoryPlanningState::invariant()
{
    assert(current_route_edge_indices.size() >= 1);
    assert(current_route_edge_indices[0] == current_edge_index);
    assert(current_edge_index < laneGraphTools.n_edges);
    assert(current_edge_path_index < laneGraphTools.n_edge_path_nodes);
    assert(delta_s_path_node_offset >= 0);
}

void VehicleTrajectoryPlanningState::apply_timestep()
{

    // calculate driving distance of this time step
    for (size_t i = 0; i < n_steps; ++i)
    {
        delta_s_path_node_offset += speed_profile[i] * dt_speed_profile;
    }

    size_t route_index = 0;
    // move our position in the lane graph by delta s
    laneGraphTools.move_along_route
    (
        current_route_edge_indices, 
        route_index,
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

    assert(speed_profile[N_STEPS_SPEED_PROFILE-1]==min_speed);
    // shift and extend speed profile
    for (int i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        if (i + n_steps < N_STEPS_SPEED_PROFILE)
        {
            // Shift the current speed profile by n_steps if possible
            speed_profile[i] = speed_profile[i + n_steps];
        }
        else
        {
            // else, stop
            speed_profile[i] = min_speed;
        }
    }
    invariant();

    t_elapsed += dt_nanos;
}

void VehicleTrajectoryPlanningState::extend_random_route(size_t n)
{
    invariant();
    vector<uint32_t> new_edge_indices;
    auto distrib = std::uniform_int_distribution<>(0,3);

    while(current_route_edge_indices.size() < n)
    {   //find next indizes for edges
        auto next_edges = laneGraphTools.find_subsequent_edges(current_route_edge_indices.back());
        
        // this could be used to specify that a vehicle should only consider a subgraph of the map to plan its paths 
        /**vector<size_t> oval;
        if(vehicle_id % 2 == 0){
           oval = vert_oval_edges_index;
        } else
        {
           oval = hor_oval_edges_index;
        }
        
        auto next_edges = laneGraphTools.follow_predefined_trajectory(current_route_edge_indices.back(), oval);**/
        assert(next_edges.size() > 0);

        if(distrib.b() != (int)next_edges.size()-1) {
            distrib = std::uniform_int_distribution<>(0,next_edges.size()-1);  // generate random integers in [0, next_edges.size()+1]
        }
        current_route_edge_indices.push_back(next_edges.at(distrib(random_gen)));
        new_edge_indices.push_back(current_route_edge_indices.back());
    }
}

TrajectoryPoint VehicleTrajectoryPlanningState::get_trajectory_point(
	        uint64_t time,	
		size_t edge_index,
		size_t edge_path_index,
		double speed
		)
{
    TrajectoryPoint trajectory_point;
    trajectory_point.t().nanoseconds(time);
    trajectory_point.px(laneGraphTools.edges_x.at(edge_index).at(edge_path_index));
    trajectory_point.py(laneGraphTools.edges_y.at(edge_index).at(edge_path_index));
    trajectory_point.vx(laneGraphTools.edges_cos.at(edge_index).at(edge_path_index) * speed);
    trajectory_point.vy(laneGraphTools.edges_sin.at(edge_index).at(edge_path_index) * speed);
    return trajectory_point;
}

// returns the delta to previous fca
uint16_t VehicleTrajectoryPlanningState::update_potential_collisions(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &other_vehicles, uint8_t winner, uint16_t old_fca)
{
    std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> m={{winner, other_vehicles[winner]}};
    uint16_t collisions_with_winner = potential_collisions(m, true);
    for (auto pair : collisions_with_opt_traj)
    {
        if (pair.first == winner)
        {
            return old_fca + (collisions_with_winner - pair.second);
        }
    }
    std::cout << "hmmmm";
    return 0; 
}

/**
 *  Compute the number of potential collisions of this vehicles optimal trajectory with the already 
 *  known trajectories
 *  Future Collision Assessment (based on Luo et al. 2016)  
 */
uint16_t VehicleTrajectoryPlanningState::potential_collisions(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &other_vehicles, bool optimal){
    vector<std::pair<size_t, size_t>> self_path;
    if(!optimal){
        self_path = get_planned_path(false);
    }
    else
    {
        collisions_with_opt_traj.clear();
        self_path = get_planned_path(true);
    }
    uint16_t sum_collisions = 0;
    uint16_t collisions_with_vehicle = 0;

    // remember if we had a collision with the same vehicle last time step so that we don't count parallel trajectories as multiple collisions
    bool last_step_collison = false;

    std::cout << "potential collisions: ";
    // Find the earliest collision
    for( auto iter = other_vehicles.begin(); iter != other_vehicles.end(); ++iter ){
        std::vector<std::pair<size_t, std::pair<size_t, size_t>>> other_path = iter->second;
        last_step_collison = false;

        if (iter->first == vehicle_id) { continue; }
        

        for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
        {   
            // It's possible that we don't have data for this index
            // We will skip it, which is unsafe, but our only option right now
            try {
                if( other_path.at(i).first != i ) { 
                    std::cout << "other_path.at(i).first is: "
                                << other_path.at(i).first 
                                << "and i is"
                                << i
                                << std::endl;
                    continue; }
            } catch (std::out_of_range const& exc) {
                continue;
            }

            if(laneGraphTools.edge_path_collisions
                [self_path[i].first]
                [self_path[i].second]
                [other_path[i].second.first]
                [other_path[i].second.second])
            {
                // check wether it's still the "same" collision
                if (!last_step_collison)
                {
                     ++collisions_with_vehicle;
                     std::cout << "t:" << i << "v:" << (int) iter->first << ";";
                }
            } else
            {
                last_step_collison = false;
            }
        }
        sum_collisions += collisions_with_vehicle;
        collisions_with_vehicle = 0;
        if (optimal)
        {
            collisions_with_opt_traj.push_back(std::make_pair(iter->first, collisions_with_vehicle));
        }
        
    }
    std::cout << std::endl;

    return sum_collisions;
}



// Change the own speed profile so as not to collide with the other_vehicles.
// other_vehicles may only contain vehicles with a higher priority
bool VehicleTrajectoryPlanningState::avoid_collisions(
    std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> other_vehicles
)
{
    int min_idx_zero_speed = std::max<int>(
        n_steps,
        static_cast<int>(n_steps-1 + ceil(speed_profile[n_steps-1] / delta_v_step))
    );
    std::cout << "min idx for zero speed: " << min_idx_zero_speed << std::endl;

    bool are_collisions_avoidable = true;
    bool is_collision_free = false;
    while(!is_collision_free && are_collisions_avoidable)
    {
        vector<std::pair<size_t, size_t>> self_path = get_planned_path(false);

        //cpm::Logging::Instance().write(1,
        //        "Loopy");
        // An index exceeding N_STEPS_SPEED_PROFILE indicates that there is no collision
        size_t earliest_collision__speed_profile_index = 1<<30;
        int colliding_vehicle_id = 0;
        int time_of_collision = 0;

        // Find the earliest collision 
        // with previous planned vehicles
        for( auto iter = other_vehicles.begin(); iter != other_vehicles.end(); ++iter ){
            std::vector<std::pair<size_t, std::pair<size_t, size_t>>> other_path = iter->second;

            for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
            {
                // It's possible that we don't have data for this index
                // We will skip it, which is unsafe, but our only option right now
                try {
                    if( other_path.at(i).first != i ) { 
                        std::cout << "no data for index: " << i << std::endl;
                        continue; }
                } catch (std::out_of_range const& exc) {
                    continue;
                }
                

                if(laneGraphTools.edge_path_collisions
                    [self_path[i].first]
                    [self_path[i].second]
                    [other_path[i].second.first]
                    [other_path[i].second.second])
                {
                    // collision detected
                    if(i < earliest_collision__speed_profile_index)
                    {
                        earliest_collision__speed_profile_index = i;
                        colliding_vehicle_id = iter->first;
                        time_of_collision = i;
                    }
                    // earliest collision for this vehicle found, stop
                    break;
                }
            }
        }

        // stop if there is no collision
        if(earliest_collision__speed_profile_index >= N_STEPS_SPEED_PROFILE) {
            is_collision_free = true;
            break;
        }


        cpm::Logging::Instance().write(2,
                "Detected potential collision; avoiding"
        );

        // fix speed profile to avoid collision,
        // beginning from 10 steps before the collision
        // TODO: Change magic numbers to named parameters/constants
        int idx_speed_reduction = earliest_collision__speed_profile_index - 10;

        // If we already are almost at min_speed at the planned index for
        // speed reduction we cannot slow down much more,
        // so we try reducing the speed 10 steps earlier
        while(idx_speed_reduction >= min_idx_zero_speed
            && speed_profile[idx_speed_reduction] < min_speed + 0.05)
        {
            idx_speed_reduction -= 10;
        }
        if (idx_speed_reduction < min_idx_zero_speed 
            && speed_profile[min_idx_zero_speed] > min_speed + 0.05)
        {
            std::cout << "trying to reduce speed at beginning, idx: " << idx_speed_reduction << std::endl;
            std::cout << "speed at minidx: " << std::setw(4) << speed_profile[min_idx_zero_speed]
            << " larger than min speed " << min_speed + 0.05 <<std::endl;
            std::cout << "setting idx to " << min_idx_zero_speed <<std::endl;
            idx_speed_reduction = min_idx_zero_speed;
        }

        if(idx_speed_reduction >= min_idx_zero_speed)
        {
            const double reduced_speed = fmax(speed_profile[idx_speed_reduction] - 0.3, min_speed);
            std::cout << 
                "Collision detected t: " << std::setw(4) << earliest_collision__speed_profile_index << 
                "  self id: " << std::setw(2)  << int(vehicle_id) << 
                "  other id: " << std::setw(2)  << colliding_vehicle_id << 
                "  Speed reduction,  t: " << std::setw(4) << idx_speed_reduction  << 
                "  cur spd: " << std::setw(4) << speed_profile[idx_speed_reduction]  << 
                "  red spd: " << reduced_speed << std::endl;
            set_speed(idx_speed_reduction, reduced_speed);
        }
        else
        {
            cpm::Logging::Instance().write(
                1,
                "Collision unavoidable between %d and %d. At %d",
                int(vehicle_id), colliding_vehicle_id, time_of_collision
            );
            std::cout << "Collision unavoidable" << std::endl;
            are_collisions_avoidable = false;
        }
    }
    // assert we haven't changed current timestep
    for (size_t i = 0; i < n_steps; i++)
    {
        assert(old_speed_profile[i]==speed_profile[i]);
    }
    return is_collision_free;
}

// sets the speed value at the index and then propagates the change
void VehicleTrajectoryPlanningState::set_speed(int idx_speed_reduction, double speed_value)
{
    assert(idx_speed_reduction >= 0);
    assert(idx_speed_reduction < N_STEPS_SPEED_PROFILE);

    speed_profile[idx_speed_reduction] = speed_value;
    
    // TODO could break earliear if fmin is the actual profile
    for (int i = 1; i < N_STEPS_SPEED_PROFILE; ++i)
    {

        int i_forward = idx_speed_reduction + i;
        int i_reverse = idx_speed_reduction - i;

        if(i_forward < N_STEPS_SPEED_PROFILE)
        {
            speed_profile[i_forward] = fmin(speed_value + i * delta_v_step, speed_profile[i_forward]);
        }

        if(i_reverse >= 8)
        {
            speed_profile[i_reverse] = fmin(speed_value + i * delta_v_step, speed_profile[i_reverse]);
        }
    }
}

void VehicleTrajectoryPlanningState::save_speed_profile()
{
    old_speed_profile = speed_profile;
}

void VehicleTrajectoryPlanningState::reset_speed_profile()
{
    reset_speed_profile(speed_profile);
}
void VehicleTrajectoryPlanningState::reset_speed_profile(
    array<double, N_STEPS_SPEED_PROFILE> &speed_profile_in_out
)
{
    reset_speed_profile(speed_profile_in_out, n_steps);
}

void VehicleTrajectoryPlanningState::reset_speed_profile(
    array<double, N_STEPS_SPEED_PROFILE> &speed_profile_in_out,
    size_t i_start
)
{
    for (; i_start < N_STEPS_SPEED_PROFILE; ++i_start)
    {
        speed_profile_in_out[i_start] = fmin(speed_profile_in_out[i_start-1] + delta_v_step, max_speed);
    }

    // Each planning horizon ends with a full stop
    int spd_prof_sz = speed_profile_in_out.size() - 1;
    for(int i = spd_prof_sz; (spd_prof_sz - i) * delta_v_step < speed_profile_in_out[i]; i--){
        speed_profile_in_out[i] = (spd_prof_sz - i) * delta_v_step;
    }
}

void VehicleTrajectoryPlanningState::revert_speed_profile()
{
    speed_profile = old_speed_profile;
}


void VehicleTrajectoryPlanningState::print_speed_profile(){
    std::cout << "Speed profile: " << std::endl;
    for (auto spd:speed_profile){std::cout << spd << ",";}
    std::cout << std::endl;
}

/**
 * Returns the planed path. Returns the optimal path if optimal is true
 */
vector<std::pair<size_t, size_t>> VehicleTrajectoryPlanningState::get_planned_path(bool optimal)
{
    vector<std::pair<size_t, size_t>> result;

    size_t future_edge_index = current_edge_index;
    size_t future_edge_path_index = current_edge_path_index;
    size_t route_index = 0;
    double delta_s = delta_s_path_node_offset;
    array<double, N_STEPS_SPEED_PROFILE> cur_speed_profile = speed_profile;
    if (optimal) {reset_speed_profile(cur_speed_profile);};

    for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        delta_s += cur_speed_profile[i] * dt_speed_profile;

        laneGraphTools.move_along_route
        (
            current_route_edge_indices,
            route_index, 
            future_edge_index, 
            future_edge_path_index, 
            delta_s
        );

        result.push_back(std::make_pair(future_edge_index, future_edge_path_index));
    }

    return result;
}

vector<TrajectoryPoint> VehicleTrajectoryPlanningState::get_planned_trajectory(int max_length) {
    vector<TrajectoryPoint> result;
    uint index = 0;
    
    for( auto point : get_planned_path(false) ) {
        if(index >= n_steps*max_length) {
            break;
	    }
	    if( index%n_steps == 0 ) {
		    result.push_back(
		        get_trajectory_point(
			        t_elapsed + index * dt_speed_profile_nanos,
			        point.first,
			        point.second,
			        speed_profile[index])
		        );
	    }
	    index++;
    }
    
    return result;
}

// combines the path with the speed profile; uses the theoretical optimal speed profile when the flag is set, the actual one otherwise
void VehicleTrajectoryPlanningState::get_lane_graph_positions(
        Trajectory &lane_graph_trajectory, bool optimal)
{

    lane_graph_trajectory.vehicle_id(vehicle_id);
    
    std::vector<LaneGraphPosition> lane_graph_positions;

    for(std::pair<size_t, size_t> path_point:get_planned_path(optimal))
    {
        LaneGraphPosition lane_graph_pos;
        lane_graph_pos.edge_index(path_point.first);
        lane_graph_pos.edge_path_index(path_point.second);

        lane_graph_positions.push_back(lane_graph_pos);
    }

    lane_graph_trajectory.lane_graph_positions(
            rti::core::vector<LaneGraphPosition>(lane_graph_positions)
    );
}

std::pair<double, double> VehicleTrajectoryPlanningState::get_position(){
    return std::make_pair (laneGraphTools.edges_x.at(current_edge_index).at(current_edge_path_index), laneGraphTools.edges_y.at(current_edge_index).at(current_edge_path_index));
} 

void VehicleTrajectoryPlanningState::debug_writeOutOwnTrajectory() {
    std::cout << "Vehicle " << static_cast<uint32_t>(vehicle_id) << std::endl;
    int index = 0;
    for(auto point : get_planned_path(false)) {
        std::cout << index << ", "; 
        std::cout << point.first << ", "; 
        std::cout << point.second << "\n"; 
        index++;
    }
    std::cout << std::endl;
}

void VehicleTrajectoryPlanningState::write_current_speed_profile(std::ofstream &stream){
    for (size_t i = 0; i < n_steps - 1; i++)
    {
        stream << speed_profile[i] << ",";
    }
    stream << speed_profile[n_steps - 1];
    stream << ";";
}

std::vector<std::pair<int, std::vector<int>>> VehicleTrajectoryPlanningState::compute_path_induced_subgraph(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer)
{
    
    std::vector<std::pair<int, std::vector<int>>> graph;

    // vehicles_buffer contains the optimal trajectories
    // encoded as (timestep, node, edge_node)
    // we need to extract the nodes and resulting edges (discarding edge_nodes)
    for (auto vehicle : vehicles_buffer)
    {
        std::vector<std::pair<size_t, std::pair<size_t, size_t>>> path = vehicle.second;
        for (size_t i=0; i < path.size()-1; i++) //we could use bigger steps
        {
            int v_1 = path.at(i).second.first;
            int v_2 = path.at(i+1).second.first;
            if (v_1 == v_2)
            {
                continue;
            }
            
            bool found = false;
            for (auto &v_i : graph)
            {
                if (v_i.first == v_1)
                {
                    found = true;
                    auto index = std::find(v_i.second.begin(), v_i.second.end(), v_2);
                    if (index == v_i.second.end())
                    {
                        v_i.second.push_back(v_2);
                    }
                    break;
                }
            }
            if (!found)
            {
                graph.push_back(std::make_pair(v_1, std::vector<int>{v_2}));
            }
        }
        
    }
    return graph;
}

// TODO: implement here: (1) compute minimum wdfvs and (2) topological ordering
std::vector<uint8_t> VehicleTrajectoryPlanningState::compute_graph_based_priorities(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer)
{

    std::cout << "graph: " ;
    std::vector<std::pair<int, std::vector<int>>> graph = compute_path_induced_subgraph(vehicles_buffer);
    for (auto v1 : graph)
    {
        std::cout << "(" << v1.first << ":";
        for (auto v2 : v1.second)
        {
            std::cout << v2  << ","; 
        }
        std::cout << "),";
        
    }
    std::cout << "\n";
    for (auto v1 : graph)
    {
        for (auto v2 : v1.second)
        {
            std::cout  << "(" << v1.first << ", " << v2  << ") ,"; 
        }
    }
    std::cout << "\n";

    // order function for each node (node, value, t_v) ... (n, v, t_v):
    std::vector<std::tuple<int, int, int>> order;
    for (auto vehicle : vehicles_buffer)
    {
        std::vector<std::pair<size_t, std::pair<size_t, size_t>>> path = vehicle.second;
        for (int i=0; i < ((int)path.size())-1; i++)
        {
            int v_1 = path.at(i).second.first;
            int v_2 = path.at(i+1).second.first;
            if (v_1 == v_2)
            {
                continue;
            }
            
            auto idx_v1 = std::find_if( order.begin(), order.end(),
                            [&v_1](const std::tuple<int, int, int>& element){ return std::get<0>(element) == v_1;} );
            auto idx_v2 = std::find_if( order.begin(), order.end(),
                            [&v_2](const std::tuple<int, int, int>& element){ return std::get<0>(element) == v_2;} );

            if (idx_v1 == order.end() && idx_v2 == order.end())
            {
                order.push_back(std::make_tuple(v_1, 1, i));
            } 
            else if (idx_v1 == order.end() && idx_v2 != order.end())
            {
                order.push_back(std::make_tuple(v_1, std::get<1>(*idx_v2) - 1, i));
            } 
            else if (idx_v1 != order.end() && idx_v2 == order.end())
            {
                order.push_back(std::make_tuple(v_2, std::get<1>(*idx_v1) + 1, i));
            } 
            else // we might want to employ some smart decisions here to avoid conflicts early in the path of a vehicle 
            {
                // i distance to current node
                // t_v distance the previously updating vehicle had
                // 3
                // t_v <= i-3 kein update

                if (i < std::get<2>(*idx_v2) + 3)// std::get<2>(*idx_v1))
                {
                        std::get<1>(*idx_v2) = std::get<1>(*idx_v1) + 1;
                        std::get<2>(*idx_v2) = i;
                }
            }
        }
    }

    std::cout << "order";
    std::cout << "{";
    for (auto pi_i : order)
    {
        std::cout << std::get<0>(pi_i) << ":" << std::get<1>(pi_i) << ",";
    }
    std::cout << "}";
    std::cout << "\n";

    // planning order: (pi(id_1), ..., pi(id_n))
    std::vector<std::pair<uint8_t, int>> pi_vec;
    int own_pi = 0;
    for (auto& vehicle : vehicles_buffer)
    {
        // path of vehicle
        std::vector<std::pair<size_t, std::pair<size_t, size_t>>> path = vehicle.second;
        // vehicle is currently on vertex:
        int v_1 = path.at(0).second.first;
        // find vertex
        auto idx_v1 = std::find_if( order.begin(), order.end(),
                            [&v_1](const std::tuple<int, int, int>& element){ return std::get<0>(element) == v_1;});
        // get vertex value: pi(v_1)
        if (vehicle_id == vehicle.first)
        {
            own_pi = std::get<1>(*idx_v1);
        }
        
        if (!pi_vec.empty() && own_pi < std::get<1>(*idx_v1))
        {
            pi_vec.insert(pi_vec.begin(), std::make_pair(vehicle.first, std::get<1>(*idx_v1)));
        }
        else
        {
            pi_vec.emplace_back(std::make_pair(vehicle.first, std::get<1>(*idx_v1)));
        }
    }

    // define consistent sorting of vehicles pi(v1)>pi(v_2) <=> a > b
    std::sort(pi_vec.begin(), pi_vec.end(), [](auto &a, auto &b) {
        return a.second > b.second || (a.second == b.second && a.first > b.first);
    });

    std::cout << "graph based prios: ";
    for (auto pi_n:pi_vec)
    {
        std::cout << (int)pi_n.first << ":" << pi_n.second << ",";
    }
    std::cout << std::endl;

    std::vector<uint8_t> prio_vec;
    for (auto pi_n:pi_vec)
    {
        prio_vec.emplace_back(pi_n.first);
    }
    return prio_vec;
    
}