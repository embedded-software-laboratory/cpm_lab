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
    extend_random_route(50);

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

    extend_random_route(50);

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
        int colliding_vehicle_id = 0;

        // Find the earliest collision
        for(std::shared_ptr<VehicleTrajectoryPlanningState> other_vehicle:other_vehicles)
        {
            vector<PathNode> other_path = other_vehicle->get_planned_path();

            assert(self_path.size() == N_STEPS_SPEED_PROFILE);
            assert(other_path.size() == N_STEPS_SPEED_PROFILE);

            for (size_t i = 0; i < N_STEPS_SPEED_PROFILE; ++i)
            {
                const double distance = min_distance_vehicle_to_vehicle(self_path[i], other_path[i]);

                if(distance < 0.06)
                {
                    // collision detected
                    if(i < earliest_collision__speed_profile_index)
                    {
                        earliest_collision__speed_profile_index = i;
                        colliding_vehicle_id = other_vehicle->vehicle_id;
                    }
                    // earliest collision for this vehicle found, stop
                    break;
                }
            }
        }

        // stop if there is no collision
        if(earliest_collision__speed_profile_index >= N_STEPS_SPEED_PROFILE) return;


        // fix speed profile to avoid collision
        int idx_speed_reduction = earliest_collision__speed_profile_index - 10;

        while(idx_speed_reduction > 0
            && speed_profile[idx_speed_reduction] < min_speed + 0.05)
        {
            idx_speed_reduction -= 100;
        }

        std::cout << 
        "Collision detected t:" << earliest_collision__speed_profile_index << 
        "  self id: " << int(vehicle_id) << 
        "  other id: " << colliding_vehicle_id << 
        "  Speed reduction,  t: " << idx_speed_reduction  << 
        "  spd: " << fmax(speed_profile[idx_speed_reduction] - 0.5, min_speed) << std::endl;

        if(idx_speed_reduction < 15)
        {
            std::cout << "Collision unavoidable between " << int(vehicle_id) << " and " << colliding_vehicle_id << std::endl;

            for(double spd:speed_profile)
            {
                std::cout << spd << ", ";
            }
            std::cout << std::endl;

            exit(1);
            return;
        }

        set_speed(idx_speed_reduction, fmax(speed_profile[idx_speed_reduction] - 0.5, min_speed));
    }
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