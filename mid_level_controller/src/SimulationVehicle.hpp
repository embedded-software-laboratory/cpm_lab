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
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <vector>

extern "C" {
#include "../../low_level_controller/vehicle_atmega2560_firmware/spi_packets.h"
}

#define INPUT_DELAY 4 // Input delay >= 0
#define MAX_NUM_VEHICLES 30

static inline double frand() { return (double(rand()))/RAND_MAX; }

class SimulationVehicle
{
    // TODO load parameters via DDS parameters
    std::vector<double> dynamics_parameters = { 1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863 };

    double px;
    double py;
    double distance = 0;
    double yaw;
    double yaw_measured;
    double speed = 0;
    double curvature = 0;

    // array to simulate time delay on inputs
    //      first element is the next to process
    //      last element is received most recently
    double motor_throttle_history[INPUT_DELAY];
    double steering_servo_history[INPUT_DELAY]; 

    cpm::Writer<VehicleObservation> writer_vehiclePoseSimulated;
    cpm::MultiVehicleReader<VehicleObservation> reader_vehiclePoseSimulated;

    SimulationIPS& simulationIPS;

    // For collision checks:
    std::map<uint64_t, Pose2D> ego_pose_history;
    std::map<uint8_t, uint64_t>  get_collisions(const uint64_t t_now, const uint8_t vehicle_id);
    

public:
    SimulationVehicle(SimulationIPS& _simulationIPS, uint8_t vehicle_id, vector<double> starting_position);

    VehicleState update(
        const double motor_throttle,
        const double steering_servo,
        const uint64_t t_now, 
        const double dt, 
        const uint8_t vehicle_id
    );

    void get_state(double& _x, double& _y, double& _yaw, double& _speed);
};