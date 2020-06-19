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
#include "VehicleCommandSpeedCurvature.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/TimerFD.hpp"

/**
 * \brief This class is used to send automated control structures to the vehicles. A prominent example would be a stop signal that is sent to 
 * all vehicles after a simulation was stopped, so that they try to freeze at their current position and do not 'drive on while slowing down'
 * 
 */
class VehicleAutomatedControl
{
private:
    //DDS data structures to send automated commands to the vehicles
    dds::domain::DomainParticipant& participant;
    dds::topic::Topic<VehicleCommandSpeedCurvature> topic_vehicleCommandSpeedCurvature;
    shared_ptr<dds::pub::DataWriter<VehicleCommandSpeedCurvature>> writer_vehicleCommandSpeedCurvature = nullptr;

    //Vehicle commands need to be sent regularly to be interpreted correctly, so e.g. a stop signal should not be sent only once (TODO: Check that)
    std::shared_ptr<cpm::TimerFD> task_loop = nullptr;
    std::mutex stop_list_mutex;
    std::map<uint32_t, uint32_t> vehicle_stop_list; //TODO: Unordered set / different approach

public:
    VehicleAutomatedControl();

    /**
     * \brief This function is used to send an immediate stop signal to all vehicles
     * \param id_list List of vehicle ids of the vehicles that should be stopped
     */
    void stop_vehicles(std::vector<uint8_t> id_list);

    /**
     * \brief This function is used to send an immediate stop signal to a single vehicle
     * \param id Vehicle id
     */
    void stop_vehicle(uint8_t id);
};