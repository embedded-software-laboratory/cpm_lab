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
#include "defaults.hpp"
#include <dds/pub/ddspub.hpp>
#include "cpm/TimerFD.hpp"
#include "Joystick.hpp"
#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <functional>

class VehicleManualControl
{
    dds::domain::DomainParticipant& participant;
    shared_ptr<Joystick> joystick = nullptr;
    std::shared_ptr<cpm::TimerFD> update_loop = nullptr;
    uint8_t vehicle_id = 0;
    
    double ref_speed = 0;
    uint64_t ref_trajectory_start_time = 0;
    int ref_trajectory_index = 0;


    dds::topic::Topic<VehicleCommandDirect> topic_vehicleCommandDirect;
    dds::topic::Topic<VehicleCommandSpeedCurvature> topic_vehicleCommandSpeedCurvature;

    shared_ptr<dds::pub::DataWriter<VehicleCommandDirect>> writer_vehicleCommandDirect = nullptr;
    shared_ptr<dds::pub::DataWriter<VehicleCommandSpeedCurvature>> writer_vehicleCommandSpeedCurvature = nullptr;

    std::function<void()> m_update_callback;

public:
    VehicleManualControl();
    void start(uint8_t vehicleId, string joystick_device_file);
    void stop();
    void set_callback(std::function<void()> update_callback) { m_update_callback = update_callback; }
    void get_state(double& throttle, double& steering);
};