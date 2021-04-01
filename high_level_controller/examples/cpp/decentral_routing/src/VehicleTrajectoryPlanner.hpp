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

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include "cpm/Logging.hpp"
#include "cpm/Writer.hpp"
#include "cpm/ReaderAbstract.hpp"

#include "VehicleCommandTrajectory.hpp"
#include "HlcCommunication.hpp"

#include "VehicleTrajectoryPlanningState.hpp"
#include "CouplingGraph.hpp"

using std::vector;

/**
 * \class VehicleTrajectoryPlanner
 * \brief Plans a random trajectory, while communicating planned trajectories with other vehicles.
 * \ingroup decentral_routing
 */
class VehicleTrajectoryPlanner
{
    //! PlanningState plans a route, treating other vehicles as fixed obstacles.
    std::unique_ptr<VehicleTrajectoryPlanningState> trajectoryPlan;
    //! The received trajectories of other vehicles get saved in here; overwritten each timestep
    std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles_buffer;
    //! Writer to broadcast our own planned trajectory
    std::unique_ptr< cpm::Writer<HlcCommunication> > writer_hlcCommunication;
    //! Reader to receive other planned trajectories
    std::unique_ptr< cpm::ReaderAbstract<HlcCommunication> > reader_hlcCommunication;
    //! Is false on the first planning step and after the planner crashed
    bool started = false;
    //! Is true, when we either couldn't find any trajectory, or we missed too many timesteps
    bool crashed = false;
    //! Becomes true when we are supposed to stop planning a timestep early
    bool volatile stopFlag = false;
    //! Becomes true, when we are currently not planning a timestep
    bool volatile isStopped = true;
    //! Time in nanoseconds, when we first started planning
    uint64_t t_start = 0;
    //! Time in nanoseconds of the current timestep
    uint64_t t_real_time = 0;
    //! Time in nanoseconds of the previous timestep
    uint64_t t_prev = 0;
    //! Length of the current timestep in nanoseconds
    uint64_t dt_nanos;
    //! Saves trajectory commands we've previously received from PlanningState
    vector<TrajectoryPoint> trajectory_point_buffer;
    //! Determines, which vehicles have priority; should be the same for all HLCs
    CouplingGraph coupling_graph;
    //! A "list" of which vehicle_ids have already sent messages to us this timestep
    std::set<uint8_t> all_received_messages; // Which vehicles have sent LaneGraphTrajs to us this timestep
    //! Counter, of how often in a row we had to abort planning before we were done
    short no_trajectory_counter = 0;
 
    // Constants, should be adjusted depending on VehicleTrajectoryPlanningState
    //! How many points we can send in one message; determined by maximum length of vector in DDS
    static constexpr int msg_max_length = 100;
    //! How many sub-divisions there are per edge on the LaneGraph; determined by geometry.hpp
    static constexpr int edge_paths_per_edge = 25;

    /**
     * \brief Read planned trajectories of vehicles that planned before us
     */
    void read_previous_vehicles();

    /**
     * \brief Read planned trajectories of vehicles that are planning at the same time as us
     */
    bool read_concurrent_vehicles();

    /**
     * \brief Read planned trajectories of vehicles that are planning after us
     */
    void wait_for_ignored_vehicles();

    /**
     * \brief Generalized method to read planned trajectories of other vehicles
     * \param vehicle_ids Which vehicle ids should be read (blocks until all these are received)
     * \param write_to_buffer When false, do not write messages to other_vehicles_buffer
     * \param final_messages_only Only read messages with the "final" message attribute
     */
    bool read_vehicles(std::set<uint8_t> vehicle_ids, bool write_to_buffer=false, bool final_messages_only=true);

    /**
     * \brief Broadcast our planned trajectory to other HLCs
     * \param is_final When false, message is iterative message, else final
     * \param has_collisions Wether this planned trajectory contains collisions
     */
    void send_plan_to_hlcs( bool is_final=true, bool has_collisions=false );

    /**
     * \brief Interpolate between two received trajectory points using fixed velocity
     * \param vehicle_id Which vehicle id these points came from
     * \param buffer_index_1 Index of first point in other_vehicle_buffer (might be negative!)
     * \param edge_1 Edge of first point
     * \param edge_index_1 Edge index of first point
     * \param buffer_index_2 Index of second point in other_vehicle_buffer
     * \param edge_2 Edge of second point
     * \param edge_index_2 Edge index of first point
     */
    void interpolate_other_vehicles_buffer(uint8_t vehicle_id,
            int buffer_index_1, int edge_1, int edge_index_1,
            int buffer_index_2, int edge_2, int edge_index_2);

    /**
     * \brief Remove all points from the trajectory_point_buffer, that are now in the past
     */
    void clear_past_trajectory_point_buffer();

    /**
     * \brief Create a trajectory command from the newest points in the trajectory_point_buffer
     * \param t_now
     */
    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);

    /**
     * \brief Debugging method; writes other_vehicles_buffer to stdout
     */
    void debug_writeOutReceivedTrajectories();

    /**
     * \brief Debugging method; writes trajectory_point_buffer to stdout and spots some common issues
     */
    void debug_analyzeTrajectoryPointBuffer();
    
public:

    /**
     * \brief Create a VehicleTrajectoryPlanner
     */
    VehicleTrajectoryPlanner();

    /**
     * \brief Returns started
     */
    bool is_started() {return started;}

    /**
     * \brief Returns crashed
     */
    bool is_crashed() {return crashed;}

    /**
     * \brief Sets the PlanningState of this Planner
     * \param vehicle (cpp-pointer to) PlanningState object
     */
    void set_vehicle(std::unique_ptr<VehicleTrajectoryPlanningState> vehicle);

    /**
     * \brief Sets coupling graph of this Planner; should be identical for all HLCs
     * \param graph CouplingGraph object
     */
    void set_coupling_graph(CouplingGraph graph) {coupling_graph = graph; };

    /**
     * \brief Set writer to send planned trajectories
     * \param writer cpm::Writer object
     */
    void set_writer(std::unique_ptr< cpm::Writer<HlcCommunication> > writer);

    /**
     * \brief Set reader to read planned trajectories
     * \param reader cpm::ReaderAbstract object
     */
    void set_reader(std::unique_ptr< cpm::ReaderAbstract<HlcCommunication> > reader);

    /**
     * \brief Plan one timestep
     * \param t_real_time Current time in nanoseconds
     * \param dt Length of this timestep
     */
    std::unique_ptr<VehicleCommandTrajectory> plan(uint64_t t_real_time, uint64_t dt);

    /**
     * \brief Stop planning of this timestep, even if we aren't finished
     */
    void stop();
};
