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

#include <list>
#include <stdint.h>

#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/Writer.hpp"
#include "geometry.hpp"
#include "VehicleModel.hpp"
#include "VehicleObservation.hpp"
#include "VehicleState.hpp"
#include "SimulationIPS.hpp"
#include <vector>

extern "C" {
#include "../../low_level_controller/vehicle_atmega2560_firmware/spi_packets.h"
}

/**
 * \file SimulationVehicle.hpp
 * \ingroup vehicle
 * \brief (Needs to be included for static functions to show up properly)
 */

/**
 * \brief TODO, Input delay >= 0
 * \ingroup vehicle
 */
#define INPUT_DELAY 4 
/**
 * \brief TODO
 * \ingroup vehicle
 */
#define MAX_NUM_VEHICLES 30

/**
 * \brief TODO
 * \ingroup vehicle
 */
static inline double frand() { return (double(rand()))/RAND_MAX; }

/**
 * \class SimulationVehicle
 * \brief TODO
 * \ingroup vehicle
 */
class SimulationVehicle
{
    //! TODO load parameters via DDS parameters
    std::vector<double> dynamics_parameters = { 1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863 };

    //! TODO
    double px;
    //! TODO
    double py;
    //! TODO
    double distance = 0;
    //! TODO
    double yaw;
    //! TODO
    double yaw_measured;
    //! TODO
    double speed = 0;
    //! TODO
    double curvature = 0;

    /**
     * \brief array to simulate time delay on inputs
     *      first element is the next to process
     *      last element is received most recently
     */
    double motor_throttle_history[INPUT_DELAY];
    //! TODO
    double steering_servo_history[INPUT_DELAY]; 

    //! TODO
    cpm::Writer<VehicleObservation> writer_vehiclePoseSimulated;
    //! TODO
    cpm::MultiVehicleReader<VehicleObservation> reader_vehiclePoseSimulated;

    //! TODO
    SimulationIPS& simulationIPS;

    //! For collision checks:
    std::map<uint64_t, Pose2D> ego_pose_history;
    /**
     * \brief TODO
     * \param t_now TODO 
     * \param vehicle_id TODO 
     */
    std::map<uint8_t, uint64_t>  get_collisions(const uint64_t t_now, const uint8_t vehicle_id);
    

public:
    /**
     * \brief TODO
     * \param _simulationIPS TODO
     * \param vehicle_id TODO
     * \param starting_position TODO
     */
    SimulationVehicle(SimulationIPS& _simulationIPS, uint8_t vehicle_id, vector<double> starting_position);

    /**
     * \brief TODO
     * \param motor_throttle TODO
     * \param steering_servo TODO
     * \param t_now TODO
     * \param dt TODO
     * \param vehicle_id TODO
     */
    VehicleState update(
        const double motor_throttle,
        const double steering_servo,
        const uint64_t t_now, 
        const double dt, 
        const uint8_t vehicle_id
    );

    /**
     * \brief TODO
     * \param _x TODO
     * \param _y TODO
     * \param _yaw TODO
     * \param _speed TODO
     */
    void get_state(double& _x, double& _y, double& _yaw, double& _speed);
};