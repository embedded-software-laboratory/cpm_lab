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

#include <memory>
#include <mutex>
#include <thread>
#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>
#include "VehicleCommandTrajectory.hpp"
#include "LaneGraphTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp"

using std::vector;

/**
 * \class VehicleTrajectoryPlanner
 * \brief TODO
 * \ingroup decentral_routing
 */
class VehicleTrajectoryPlanner
{
    //! TODO
    std::shared_ptr<VehicleTrajectoryPlanningState> trajectoryPlan;
    //! TODO
    std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles_buffer;
    //! TODO
    std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectory> > writer_laneGraphTrajectory;
    //! TODO
    std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectory> > reader_laneGraphTrajectory;
    //! TODO
    bool started = false;
    //! TODO
    bool crashed = false;
    //! TODO
    uint64_t t_start = 0;
    //! TODO
    uint64_t t_real_time = 0;
    //! TODO
    uint64_t t_planning;
    //! TODO
    std::mutex mutex;
    //! TODO
    std::thread planning_thread;
    //! TODO
    const uint64_t dt_nanos;
    //! TODO
    vector<TrajectoryPoint> trajectory_point_buffer;

    /**
     * \brief TODO
     */
    void read_other_vehicles();

    /**
     * \brief TODO
     */
    void apply_timestep();

    /**
     * \brief TODO
     * \param trajectory TODO
     */
    void write_trajectory( LaneGraphTrajectory trajectory );

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
 
    // Constants, should be adjusted depending on VehicleTrajectoryPlanningState
    //! Maximum length of RTI DDS msg
    static constexpr int msg_max_length = 100;
    //! Constant from geometry.hpp
    static constexpr int edge_paths_per_edge = 25;
    //! For each dt_nanos, each HLC has 5 points planned
    static constexpr int timesteps_per_planningstep = 5;
    
public:
    /**
     * \brief TODO
     * \param dt_nanos TODO
     */
    VehicleTrajectoryPlanner(uint64_t dt_nanos);

    /**
     * \brief TODO
     */
    ~VehicleTrajectoryPlanner();

    /**
     * \brief TODO
     * \param t_now TODO
     */
    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);

    /**
     * \brief TODO
     * \param t TODO
     */
    void set_real_time(uint64_t t);

    /**
     * \brief TODO
     */
    bool is_started() {return started;}

    /**
     * \brief TODO
     */
    bool is_crashed() {return crashed;}

    /**
     * \brief TODO
     * \param vehicle TODO
     */
    void set_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle);

    /**
     * \brief TODO
     * \param writer TODO
     */
    void set_writer(std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectory> > writer);

    /**
     * \brief TODO
     * \param reader TODO
     */
    void set_reader(std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectory> > reader);

    /**
     * \brief TODO
     */
    void start();

};
