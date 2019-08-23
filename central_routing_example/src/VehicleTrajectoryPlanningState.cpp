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

    double acceleration = ref_acceleration;
    if(speed_profile[0] >= max_speed) // TODO redo this
    {
        acceleration = 0;
    }
    const double dt = (dt_nanos*1e-9);
    speed_profile[0] += dt * acceleration;
    const double delta_s = dt * speed_profile[0]; // TODO delta s calculation based on speed profile

    laneGraphTools.move_along_route
    (
        current_route_edge_indices, 
        current_edge_index, 
        current_edge_path_index, 
        delta_s
    );

    while( !current_route_edge_indices.empty()
        && current_route_edge_indices.at(0) != current_edge_index)
    {
        current_route_edge_indices.erase(current_route_edge_indices.begin());
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