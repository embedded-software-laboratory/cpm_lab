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
#include "LaneGraphTrajectory.hpp"

#include "VehicleTrajectoryPlanningState.hpp"
#include "CouplingGraph.hpp"

using std::vector;

class VehicleTrajectoryPlanner
{
    std::unique_ptr<VehicleTrajectoryPlanningState> trajectoryPlan;
    std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles_buffer;
    std::unique_ptr< cpm::Writer<LaneGraphTrajectory> > writer_laneGraphTrajectory;
    std::unique_ptr< cpm::ReaderAbstract<LaneGraphTrajectory> > reader_laneGraphTrajectory;
    bool started = false;
    bool crashed = false;
    bool volatile stopFlag = false;
    bool volatile isStopped = true;
    uint64_t t_start = 0;
    uint64_t t_real_time = 0;
    uint64_t t_prev = 0;
    std::mutex mutex;
    std::thread planning_thread;
    uint64_t dt_nanos;
    const int planning_horizont = 5;
    const uint64_t dt_keep_past_trajectories = 1000000000;
    vector<TrajectoryPoint> trajectory_point_buffer;
    CouplingGraph coupling_graph;
    std::set<uint8_t> messages_received; // Which vehicles have sent LaneGraphTrajs to us this timestep
    short no_trajectory_counter = 0;
 
    // Constants, should be adjusted depending on VehicleTrajectoryPlanningState
    static constexpr int msg_max_length = 100; // Maximum length of RTI DDS msg
    static constexpr int edge_paths_per_edge = 25; // Constant from geometry.hpp

    void read_prev_vehicles();
    bool read_concurrent_vehicles();
    bool wait_for_other_vehicles();
    void write_trajectory( LaneGraphTrajectory trajectory );
    void interpolate_other_vehicles_buffer(uint8_t vehicle_id,
            int buffer_index_1, int edge_1, int edge_index_1,
            int buffer_index_2, int edge_2, int edge_index_2);
    void clear_past_trajectory_point_buffer();
    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);
    void debug_writeOutReceivedTrajectories(); // Debugging method
    void debug_analyzeTrajectoryPointBuffer(); // Debugging method
    
public:

    VehicleTrajectoryPlanner();
    ~VehicleTrajectoryPlanner();
    void set_real_time(uint64_t t);
    bool is_started() {return started;}
    bool is_crashed() {return crashed;}
    void set_vehicle(std::unique_ptr<VehicleTrajectoryPlanningState> vehicle);
    void set_coupling_graph(CouplingGraph graph) {coupling_graph = graph; };
    void set_writer(std::unique_ptr<cpm::Writer<LaneGraphTrajectory> > writer);
    void set_reader(std::unique_ptr<cpm::ReaderAbstract<LaneGraphTrajectory> > reader);
    std::unique_ptr<VehicleCommandTrajectory> plan(uint64_t t_real_time, uint64_t dt);
    void stop();
    void start(){started = true;};

};
