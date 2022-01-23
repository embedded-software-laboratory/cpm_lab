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
#include "Trajectory.hpp"

using std::vector;
using std::array;

//TODO: Replace this with a proper variable
#define N_STEPS_SPEED_PROFILE (200)


/**
 * \class VehicleTrajectoryPlanningState
 * \brief Calculates and saves which trajectory will be driven
 * \ingroup decentral_routing
 */
class VehicleTrajectoryPlanningState
{
    //! Which vehicle id this belongs to
    uint8_t vehicle_id = 0;
    //! Which edge index we are currently at (NOT the current vehicle position)
    size_t current_edge_index = 0;
    //! Which edge path index we are currently at (NOT the current vehicle position)
    size_t current_edge_path_index = 0;
    //! How much beyond the current_edge_path_index we are (in m?)
    double delta_s_path_node_offset = 0;
    //! List of edge indices, which we will plan to next
    vector<size_t> current_route_edge_indices;

    //! How much time has elapsed between when we started planning, and the position we're currently planning for
    uint64_t t_elapsed = 0;

    //! Acceleration used for speeding up/slowing down in m/s^2
    static constexpr double ref_acceleration = 0.8;
    //! Maximum speed in m/s
    static constexpr double max_speed = 1.4;
    //! Minimum speed in m/s
    static constexpr double min_speed = 0.0;
    //! We subdivide each planning steps into time segments this large (ns)
    static constexpr uint64_t dt_speed_profile_nanos = 5*10000000ull;
    //! dt_speed_profile_nanos converted to seconds
    static constexpr double dt_speed_profile = (dt_speed_profile_nanos * 1e-9);
    //! Maximum change in velocity per small timestep
    static constexpr double delta_v_step = ref_acceleration * dt_speed_profile;

    //! Determines, which velocity we need to have at which time; one entry per dt_speed_profile
    array<double, N_STEPS_SPEED_PROFILE> speed_profile;
    //! Speed profile of t-1
    array<double, N_STEPS_SPEED_PROFILE> old_speed_profile;
    //! Pointer to real time
    uint64_t* t_real_time;

    //! Save collisions with other vehicles optimal trajectories for efficient fca update when the winner planned
    std::vector<std::pair<uint8_t, uint16_t>> collisions_with_opt_traj;

    /**
     * \brief Sanity check, if we are in a valid state
     */
    void invariant();

    /**
     * \brief Extends current_route_edge_indices with random indices only length n is reached
     * \param n Length that current_route_edge_indices should be extended to
     */
    void extend_random_route(size_t n);

    /**
     * \brief TODO
     * 
     */
    void write_path(std::vector<uint32_t> path);

    /**
     * \brief Returns the edge_index/edge_path_index for each speed profile step, in order
     */
    vector<std::pair<size_t, size_t>> get_planned_path(bool optimal);

    /**
     * \brief Set speed at index, and adjust the previous/following speeds as well
     * \param idx_speed_reduction Index of where we want to set the speed
     * \param speed_value Value of speed (m/s)
     */
    void set_speed(int idx_speed_reduction, double speed_value);

public:
    /**
     * \brief Create a new VehicleTrajectoryPlanningState object
     * \param _vehicle_id Which vehicle id the PlanningState belongs to
     * \param _edge_index Which edge index the vehicle is starting on
     * \param _edge_path_index Which edge path index the vehicle is starting on
     */
    VehicleTrajectoryPlanningState(
        uint8_t _vehicle_id,
        size_t _edge_index,
        size_t _edge_path_index,
        uint64_t dt_nanos
    );

    /**
     * \brief Write out planned trajectory into a HlcCommunication object
     * \param lane_graph_trajectory HlcCommunication object to write into
     */
    void get_lane_graph_positions(Trajectory &lane_graph_trajectory, bool optimal);

    /**
     * \brief Returns a single TrajectoryPoint object, with the given parameters
     * \param time Time, when we reach this point (ns)
     * \param edge_index Edge index of point
     * \param edge_path_index Edge path index of point
     * \param speed Which speed we should have at this point
     */
    TrajectoryPoint get_trajectory_point(uint64_t time, size_t edge_index, size_t edge_path_index, double speed);

    /**
     * \brief Returns the currently planned trajectory as a "list" of TrajectoryPoints
     * \param max_length Maximum length of trajectory that should be returned
     * \param dt_nanos How much time each point should be from the next
     */
    vector<TrajectoryPoint> get_planned_trajectory(int max_length);

    /**
     * \brief Advance the planning by a timestep
     */
    void apply_timestep();

    /**
     * \brief Return length of one speed profile step
     */
    uint64_t get_dt_speed_profile_nanos() {return dt_speed_profile_nanos;};

    /**
     * \brief Debug method;
     */
    void debug_writeOutOwnTrajectory(); // Debugging method

    /**
    *  \brief Compute the number of potential collisions in on the already known trajectories
    *  (Future Collision Assessment) 
    */
    uint16_t potential_collisions(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &other_vehicles, bool optimal);

    /**
    *  \brief Updates the number of potential collisions by recomputing the potential collisions with the winner of the last plan step
    *  (Future Collision Assessment) 
    */
    uint16_t update_potential_collisions(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &other_vehicles, uint8_t winner, uint16_t old_fca);

    /**
     * TODO
     */
    std::pair<double, double> get_position();

    /**
     * \brief Change the own speed profile so as not to collide with the other_vehicles.
     * \param other_vehicles
     * \return True if successful and False otherwise.
     */
    bool avoid_collisions(
        std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> other_vehicles
    );

    /**
     * \brief Returns the corresponding vehicle id
     */
    uint8_t get_vehicle_id(){return vehicle_id;}

    /**
     * \brief Saves the speed profile into old_speed_profile
     **/
    void save_speed_profile();

    /**
     * \brief Speed back up and reset planned braking (except for the safety stop)
     **/
    void reset_speed_profile();
    /**
     * \brief Speed back up and reset planned braking (except for the safety stop)
     **/
    void reset_speed_profile(array<double, N_STEPS_SPEED_PROFILE> &speed_profile_in_out);
    /**
     * \brief Speed back up and reset planned braking (except for the safety stop)
     **/
    void reset_speed_profile(
        array<double, N_STEPS_SPEED_PROFILE> &speed_profile_in_out,
        size_t i_start
    );

    /**
     * \brief The speed profile gets reverted to the one saved by the last save_speed_profile() call
     **/
    void revert_speed_profile();

    /**
     * 
     *
     **/
    void print_speed_profile();
    
    /*
    */
   void write_current_speed_profile(std::ofstream &stream);

    /**
    * \brief compute the subgraph induced by the paths of the vehicles
    * \return nodes
    **/
   std::vector<std::pair<int, std::vector<int>>> compute_path_induced_subgraph(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer);


    /**
     * \brief computes a vertex ordering based priority vector
     * \return ^
     **/
    std::vector<uint8_t> compute_graph_based_priorities(std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer);

    //! Length of the current timestep in nanoseconds
    uint64_t const dt_nanos;

    //! Number of speed_profile steps whose duration sum up to dt_nanos
    size_t const n_steps;
};
