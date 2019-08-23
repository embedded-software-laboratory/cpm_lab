#include "VehicleTrajectoryPlanningState.hpp"

#include "lane_graph_tools.hpp"


VehicleTrajectoryPlanningState::VehicleTrajectoryPlanningState(
    uint8_t _vehicle_id,
    size_t _edge_index,
    size_t _edge_path_index)
:vehicle_id(_vehicle_id)
,current_edge_index(_edge_index)
,current_edge_path_index(_edge_path_index)
,current_route_edge_indices({_edge_index})
{
    extend_random_route(15);

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
}

void VehicleTrajectoryPlanningState::apply_timestep(uint64_t dt_nanos)
{
    uint64_t n_steps = dt_nanos / dt_speed_profile_nanos;
    assert(n_steps * dt_speed_profile_nanos == dt_nanos); // major timestep is multiple of minor timestep
    assert(n_steps * 2 < N_STEPS_SPEED_PROFILE);
    assert(n_steps >= 1);

    // calculate driving distance of this time step
    double delta_s = 0;
    for (size_t i = 0; i < n_steps; ++i)
    {
        delta_s += speed_profile[i] * dt_speed_profile;
    }

    // move our position in the lane graph by delta s
    laneGraphTools.move_along_route
    (
        current_route_edge_indices, 
        current_edge_index, 
        current_edge_path_index, 
        delta_s
    );

    // delete old route edge(s)
    while( !current_route_edge_indices.empty()
        && current_route_edge_indices.at(0) != current_edge_index)
    {
        current_route_edge_indices.erase(current_route_edge_indices.begin());
    }

    extend_random_route(15);

    // shift and extend speed profile
    for (int i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        if(i + n_steps < N_STEPS_SPEED_PROFILE)
        {
            speed_profile[i] = speed_profile[i + n_steps];
        }
        else
        {
            speed_profile[i] = fmin(speed_profile[i-1] + delta_v_step, max_speed);
        }
    }

    invariant();
}

void VehicleTrajectoryPlanningState::extend_random_route(size_t n)
{
    invariant();

    while(current_route_edge_indices.size() < n)
    {
        auto next_edges = laneGraphTools.find_subsequent_edges(current_route_edge_indices.back());
        assert(next_edges.size() > 0);
        current_route_edge_indices.push_back(next_edges.at(rand() % next_edges.size()));
    }
}

VehicleCommandTrajectory VehicleTrajectoryPlanningState::get_trajectory_command(uint64_t t_now)
{
    TrajectoryPoint trajectory_point;
    trajectory_point.t().nanoseconds(t_now + 1200000000ull);
    trajectory_point.px(laneGraphTools.edges_x.at(current_edge_index).at(current_edge_path_index));
    trajectory_point.py(laneGraphTools.edges_y.at(current_edge_index).at(current_edge_path_index));
    trajectory_point.vx(laneGraphTools.edges_cos.at(current_edge_index).at(current_edge_path_index) * speed_profile[0]);
    trajectory_point.vy(laneGraphTools.edges_sin.at(current_edge_index).at(current_edge_path_index) * speed_profile[0]);
    VehicleCommandTrajectory vehicleCommandTrajectory;
    vehicleCommandTrajectory.vehicle_id(vehicle_id);
    vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));
    return vehicleCommandTrajectory;
}



// Change the own speed profile so as not to collide with the other_vehicles.
void VehicleTrajectoryPlanningState::avoid_collisions(
    vector< std::shared_ptr<VehicleTrajectoryPlanningState> > other_vehicles)
{
    // TODO termination condition
    // for now: if this gets stuck the vehicles will stop, because they wont get a new command
    while(1)
    {
        vector<PathNode> self_path = get_planned_path();

        // An index exceeding N_STEPS_SPEED_PROFILE indicates that there is no collision
        int earliest_collision__speed_profile_index = 1<<30;

        // Find the earliest collision
        for(std::shared_ptr<VehicleTrajectoryPlanningState> other_vehicle:other_vehicles)
        {
            vector<PathNode> other_path = other_vehicle->get_planned_path();

            assert(self_path.size() == N_STEPS_SPEED_PROFILE);
            assert(other_path.size() == N_STEPS_SPEED_PROFILE);

            for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
            {
                const double distance = min_distance_vehicle_to_vehicle(self_path[i], other_path[i]);

                if(distance < 0.1)
                {
                    // collision detected
                    if(i < earliest_collision__speed_profile_index)
                    {
                        earliest_collision__speed_profile_index = i;
                    }
                    // earliest collision for this vehicle found, stop
                    break;
                }
            }
        }

        // stop if there is no collision
        if(earliest_collision__speed_profile_index > N_STEPS_SPEED_PROFILE) return;

        // TODO fix speed profile to avoid collision
        std::cout << "Collision detected" << std::endl;
        return;
    }
}

vector<PathNode> VehicleTrajectoryPlanningState::get_planned_path()
{
    vector<PathNode> result;
    
    double delta_s = 0;
    for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
    {
        delta_s += speed_profile[i] * dt_speed_profile;

        size_t future_edge_index = current_edge_index;
        size_t future_edge_path_index = current_edge_path_index;

        laneGraphTools.move_along_route
        (
            current_route_edge_indices, 
            future_edge_index, 
            future_edge_path_index, 
            delta_s
        );

        result.emplace_back(
            laneGraphTools.edges_x.at(future_edge_index).at(future_edge_path_index),
            laneGraphTools.edges_y.at(future_edge_index).at(future_edge_path_index),
            laneGraphTools.edges_cos.at(future_edge_index).at(future_edge_path_index),
            laneGraphTools.edges_sin.at(future_edge_index).at(future_edge_path_index)
        );
    }

    return result;
}