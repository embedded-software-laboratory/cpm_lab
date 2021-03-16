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
 * \brief TODO
 * \ingroup decentral_routing
 */
class VehicleTrajectoryPlanner
{
    //! TODO
    std::unique_ptr<VehicleTrajectoryPlanningState> trajectoryPlan;
    //! TODO
    std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles_buffer;
    //! TODO
    std::unique_ptr< cpm::Writer<HlcCommunication> > writer_hlcCommunication;
    //! TODO
    std::unique_ptr< cpm::ReaderAbstract<HlcCommunication> > reader_hlcCommunication;
    bool started = false;
    //! TODO
    bool crashed = false;
    //! TODO
    bool volatile stopFlag = false;
    //! TODO
    bool volatile isStopped = true;
    //! TODO
    uint64_t t_start = 0;
    //! TODO
    uint64_t t_real_time = 0;
    //! TODO
    uint64_t t_prev = 0;
    //! TODO
    std::mutex mutex;
    //! TODO
    std::thread planning_thread;
    //! TODO
    uint64_t dt_nanos;
    //! TODO
    const int planning_horizont = 5;
    //! TODO
    const uint64_t dt_keep_past_trajectories = 1000000000;
    //! TODO
    vector<TrajectoryPoint> trajectory_point_buffer;
    //! TODO
    CouplingGraph coupling_graph;
    //! TODO
    std::set<uint8_t> all_received_messages; // Which vehicles have sent LaneGraphTrajs to us this timestep
    //! TODO
    short no_trajectory_counter = 0;
 
    // Constants, should be adjusted depending on VehicleTrajectoryPlanningState
    //! TODO
    static constexpr int msg_max_length = 100; // Maximum length of RTI DDS msg
    //! TODO
    static constexpr int edge_paths_per_edge = 25; // Constant from geometry.hpp

    /**
     * \brief TODO
     */
    void read_previous_vehicles();

    /**
     * \brief TODO
     */
    bool read_concurrent_vehicles();

    /**
     * \brief TODO
     */
    void wait_for_ignored_vehicles();

    /**
     * \brief TODO
     * \param vehicle_ids TODO
     * \param write_to_buffer TODO
     * \param final_messages_only TODO
     */
    bool read_vehicles(std::set<uint8_t> vehicle_ids, bool write_to_buffer=false, bool final_messages_only=true);

    /**
     * \brief TODO
     * \param is_final TODO
     * \param has_collisions TODO
     */
    void send_plan_to_hlcs( bool is_final=true, bool has_collisions=false );
    /**
     * \brief TODO
     * \param trajectory
     */
    void write_trajectory( HlcCommunication trajectory );

    /**
     * \brief TODO
     * \param vehicle_id TODO
     * \param buffer_index_1 TODO
     * \param edge_1 TODO
     * \param edge_index_1 TODO
     * \param buffer_index_2 TODO
     * \param edge_2 TODO
     * \param edge_index_2 TODO
     */
    void interpolate_other_vehicles_buffer(uint8_t vehicle_id,
            int buffer_index_1, int edge_1, int edge_index_1,
            int buffer_index_2, int edge_2, int edge_index_2);

    /**
     * \brief TODO
     */
    void clear_past_trajectory_point_buffer();

    /**
     * \brief TODO
     * \param t_now
     */
    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);

    /**
     * \brief TODO
     */
    void debug_writeOutReceivedTrajectories(); // Debugging method

    /**
     * \brief TODO
     */
    void debug_analyzeTrajectoryPointBuffer(); // Debugging method
    
public:

    /**
     * \brief TODO
     */
    VehicleTrajectoryPlanner();
    ~VehicleTrajectoryPlanner();

    /**
     * \brief TODO
     * \param t
     */
    void set_real_time(uint64_t t);

    /**
     * \brief TODO
     * \param started TODO
     */
    bool is_started() {return started;}

    /**
     * \brief TODO
     * \param crashed TODO
     */
    bool is_crashed() {return crashed;}

    /**
     * \brief TODO
     * \param vehicle TODO
     */
    void set_vehicle(std::unique_ptr<VehicleTrajectoryPlanningState> vehicle);

    /**
     * \brief TODO
     * \param graph TODO
     */
    void set_coupling_graph(CouplingGraph graph) {coupling_graph = graph; };

    /**
     * \brief TODO
     * \param writer TODO
     */
    void set_writer(std::unique_ptr< cpm::Writer<HlcCommunication> > writer);

    /**
     * \brief TODO
     * \param reader TODO
     */
    void set_reader(std::unique_ptr< cpm::ReaderAbstract<HlcCommunication> > reader);

    /**
     * \brief TODO
     * \param t_real_time TODO
     * \param dt TODO
     */
    std::unique_ptr<VehicleCommandTrajectory> plan(uint64_t t_real_time, uint64_t dt);

    /**
     * \brief TODO
     */
    void stop();

    /**
     * \brief TODO
     */
    void start(){started = true;};

};
