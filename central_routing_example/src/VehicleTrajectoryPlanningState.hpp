#pragma once

#include <vector>
#include <cstdint>
#include "VehicleCommandTrajectory.hpp"
using std::vector;

class VehicleTrajectoryPlanningState
{
    uint8_t vehicle_id = 0;
    size_t current_edge_index = 0;
    size_t current_edge_path_index = 0;
    double current_speed = 0;
    vector<size_t> current_route_edge_indices;

    void invariant();
public:

    VehicleTrajectoryPlanningState(){}
    VehicleTrajectoryPlanningState(
        uint8_t _vehicle_id,
        size_t _edge_index,
        size_t _edge_path_index);

    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);
    void extend_random_route(size_t n);
    void apply_timestep(uint64_t dt_nanos);
};
