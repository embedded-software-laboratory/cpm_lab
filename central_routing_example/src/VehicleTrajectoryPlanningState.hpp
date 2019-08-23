#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include "VehicleCommandTrajectory.hpp"
using std::vector;
using std::array;

#define N_STEPS_SPEED_PROFILE (100)

struct PathNode
{
    double x;
    double y;
    double cos_yaw;
    double sin_yaw;
    PathNode(){}
    PathNode(double x, double y, double cos_yaw, double sin_yaw)
    :x(x), y(y), cos_yaw(cos_yaw), sin_yaw(sin_yaw){}
};

class VehicleTrajectoryPlanningState
{
    uint8_t vehicle_id = 0;
    size_t current_edge_index = 0;
    size_t current_edge_path_index = 0;
    vector<size_t> current_route_edge_indices;

    static constexpr double ref_acceleration = 0.8;
    static constexpr double max_speed = 1.4;
    static constexpr double min_speed = 0.5;
    static constexpr uint64_t dt_speed_profile_nanos = 40000000ull;
    static constexpr double dt_speed_profile = (dt_speed_profile_nanos * 1e-9);
    static constexpr double delta_v_step = ref_acceleration * dt_speed_profile;

    array<double, N_STEPS_SPEED_PROFILE> speed_profile;

    void invariant();
    void extend_random_route(size_t n);
    vector<PathNode> get_planned_path();
public:

    VehicleTrajectoryPlanningState(){}
    VehicleTrajectoryPlanningState(
        uint8_t _vehicle_id,
        size_t _edge_index,
        size_t _edge_path_index);

    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);
    void apply_timestep(uint64_t dt_nanos);

    // Change the own speed profile so as not to collide with the other_vehicles.
    void avoid_collisions(vector< std::shared_ptr<VehicleTrajectoryPlanningState> > other_vehicles);
};
