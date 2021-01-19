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
#include <stdint.h>
#include "defaults.hpp"
#include "Pose2D.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/TimerFD.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"

/**
 * \class TrajectoryCommand
 * \brief Using this class, trajectories from 2D paths can be created and sent to the vehicles. 
 * Used to create and send trajectories from paths drawn in the MapView
 * \ingroup lcc
 */
class TrajectoryCommand
{
    //! Mutex for accessing vehicle_trajectories
    std::mutex _mutex;
    //! Map that stores vehicle trajectories for multiple vehicles (vehicle ID as identifier, uint8_t)
    map<uint8_t, vector<TrajectoryPoint>> vehicle_trajectories;
    /**
     * \brief Timer to regularly send new trajectory data to the vehicles (may send nothing in between, if no such data exists). 
     * Calls send_trajectory in the callback. 
     * Does not respond to stop signals as it can be used independt of running simulations 
     * (as vehicle paths can always be drawn, also to align vehicles before a simulation).
     */
    std::shared_ptr<cpm::TimerFD> timer;

    //! Writer to send trajectories to the vehicles
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory;

    /**
     * \brief Function to send a current trajectory, created from the drawn path in the LCC's MapView using set_path and stored
     * in vehicle_trajectories. Takes the current time as input to determine which part of the trajectory to send.
     * May not send anything if currently no new trajectories exist.
     * 
     * \param t_now Current time in nanoseconds, as given by the timer
     */
    void send_trajectory(uint64_t t_now);

public:
    /**
     * \brief Constructor, sets up the writer, starts the timer (async. / in a new thread internally)
     */
    TrajectoryCommand();

    /**
     * \brief Stops the timer (thread)
     */
    ~TrajectoryCommand();

    /**
     * \brief Translate a path drawn in the LCC's MapView for a vehicle to a trajectory for the same vehicle and store it in vehicle_trajectories
     * \param vehicle_id The vehicle to create the trajectory points for
     * \param path The path drawn for the vehicle in the LCC's MapView
     */
    void set_path(uint8_t vehicle_id, std::vector<Pose2D> path);

    /**
     * \brief Erases all created vehicle trajectories in vehicle_trajectories for the given vehicle.
     * Thus, all previously created trajectories are no longer executed and thus stopped for that vehicle.
     * \param vehicle_id The vehicle to stop the trajectory "execution" for
     */
    void stop(uint8_t vehicle_id);

    /**
     * \brief  Erases all created vehicle trajectories in vehicle_trajectories.
     * Thus, all previously created trajectories are no longer executed and thus stopped.
     */
    void stop_all();
    
};