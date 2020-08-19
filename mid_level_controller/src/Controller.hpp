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

#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include <map>
#include <memory>
#include <mutex>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "MpcController.hpp"
#include "TrajectoryInterpolation.hpp"

extern "C" {
    #include "../../low_level_controller/vehicle_atmega2560_firmware/spi_packets.h"
}

#define TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE 1500

enum class ControllerState
{
    Stop,
    Direct,
    SpeedCurvature,
    Trajectory
};

class Controller
{
    MpcController mpcController;

    std::function<uint64_t()> m_get_time;

    cpm::VehicleIDFilteredTopic<VehicleCommandDirect> topic_vehicleCommandDirect;
    std::unique_ptr< cpm::Reader<VehicleCommandDirect> > reader_CommandDirect;

    cpm::VehicleIDFilteredTopic<VehicleCommandSpeedCurvature> topic_vehicleCommandSpeedCurvature;
    std::unique_ptr< cpm::Reader<VehicleCommandSpeedCurvature> > reader_CommandSpeedCurvature;

    cpm::VehicleIDFilteredTopic<VehicleCommandTrajectory> topic_vehicleCommandTrajectory;
    std::unique_ptr< cpm::Reader<VehicleCommandTrajectory> > reader_CommandTrajectory;

    VehicleState m_vehicleState;

    VehicleCommandDirect m_vehicleCommandDirect;
    VehicleCommandSpeedCurvature m_vehicleCommandSpeedCurvature;
    VehicleCommandTrajectory m_vehicleCommandTrajectory;
    
    uint8_t vehicle_id;

    ControllerState state = ControllerState::Stop;

    const uint64_t command_timeout = 1000000000ull;

    double speed_throttle_error_integral = 0;

    std::mutex command_receive_mutex;

    // TODO remove linear trajectory controller related stuff, once the MPC works well
    double trajectory_controller_lateral_P_gain;
    double trajectory_controller_lateral_D_gain;
    void trajectory_controller_linear(uint64_t t_now, double &motor_throttle_out, double &steering_servo_out);

    void update_remote_parameters();

    double speed_controller(const double speed_measured, const double speed_target);

    void receive_commands(uint64_t t_now);

    std::shared_ptr<TrajectoryInterpolation> interpolate_trajectory_command(uint64_t t_now);

    // Trajectory tacking statistics
    void trajectory_tracking_statistics_update(uint64_t t_now);
    double trajectory_tracking_statistics_longitudinal_errors  [TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE];
    double trajectory_tracking_statistics_lateral_errors       [TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE];
    size_t trajectory_tracking_statistics_index = 0;

public:
    Controller(uint8_t vehicle_id, std::function<uint64_t()> _get_time);
    void update_vehicle_state(VehicleState vehicleState);
    void get_control_signals(uint64_t stamp_now, double &motor_throttle, double &steering_servo);

    /**
     * \brief Uses a P controller based on speed_controller (which uses a PI controller) to calculate the correct values for stopping the car (only for that)
     * \param motor_throttle Motor setting required to stop
     * \param steering_servo Steering value
     */
    void get_stop_signals(double &motor_throttle, double &steering_servo);
    void reset(); //Resets Reader and trajectory list, sets the current state to stop
};
