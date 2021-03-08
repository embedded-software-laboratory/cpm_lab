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

#pragma once

#include <vector>
#include <array>
#include <utility>
#include <cstdint>
#include "cpm/Logging.hpp"
#include "VehicleCommandTrajectory.hpp"
using std::vector;
using std::array;

#define N_STEPS_SPEED_PROFILE (3000)


/**
 * \class VehicleTrajectoryPlanningState
 * \brief TODO
 * \ingroup central_routing
 */
class VehicleTrajectoryPlanningState
{
    //! TODO
    uint8_t vehicle_id = 0;
    //! TODO
    size_t current_edge_index = 0;
    //! TODO
    size_t current_edge_path_index = 0;
    //! TODO
    double delta_s_path_node_offset = 0;
    //! TODO
    vector<size_t> current_route_edge_indices;

    //! TODO
    uint64_t t_elapsed = 0;

    //! TODO
    static constexpr double ref_acceleration = 0.8;
    //! TODO
    static constexpr double max_speed = 1.4;
    //! TODO
    static constexpr double min_speed = 0.5;
    //! TODO
    static constexpr uint64_t dt_speed_profile_nanos = 16000000ull;
    //! TODO
    static constexpr double dt_speed_profile = (dt_speed_profile_nanos * 1e-9);
    //! TODO
    static constexpr double delta_v_step = ref_acceleration * dt_speed_profile;

    //! TODO
    array<double, N_STEPS_SPEED_PROFILE> speed_profile;

    /**
     * \brief TODO
     */
    void invariant();

    /**
     * \brief TODO
     * \param n
     */
    void extend_random_route(size_t n);

    /**
     * \brief TODO
     */
    vector<std::pair<size_t, size_t>> get_planned_path();

    /**
     * \brief TODO
     * \param idx_speed_reduction
     * \param speed_value
     */
    void set_speed(int idx_speed_reduction, double speed_value);

public:
    /**
     * \brief Constructor TODO
     */
    VehicleTrajectoryPlanningState(){}

    /**
     * \brief Constructor TODO
     * \param _vehicle_id
     * \param _edge_index
     * \param _edge_path_index
     */
    VehicleTrajectoryPlanningState(
        uint8_t _vehicle_id,
        size_t _edge_index,
        size_t _edge_path_index);

    /**
     * \brief TODO
     */
    TrajectoryPoint get_trajectory_point();

    /**
     * \brief TODO
     * \param dt_nanos
     */
    void apply_timestep(uint64_t dt_nanos);

    /**
     * \brief Change the own speed profile so as not to collide with the other_vehicles.
     * \param other_vehicles
     */
    bool avoid_collisions(vector< std::shared_ptr<VehicleTrajectoryPlanningState> > other_vehicles);

    /**
     * \brief TODO
     */
    uint8_t get_vehicle_id(){return vehicle_id;}
};
