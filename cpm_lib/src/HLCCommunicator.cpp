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

#include "cpm/HLCCommunicator.hpp"
HLCCommunicator::HLCCommunicator(std::vector<uint8_t> _vehicle_ids, int middleware_domain, std::string qos_file, std::string qos_profile):
    vehicle_ids(_vehicle_ids),
    p_local_comms_participant(
            new cpm::Participant(
                middleware_domain,
                qos_file,
                qos_profile
            )
    ),
    writer_readyStatus(
            p_local_comms_participant->get_participant(),
            "readyStatus",
            true,
            true,
            true),
    writer_stopRequest(
            p_local_comms_participant->get_participant(),
            "stopRequest"),
    reader_vehicleStateList(
            p_local_comms_participant->get_participant(),
            "vehicleStateList"),
    reader_systemTrigger(
            p_local_comms_participant->get_participant(),
            "systemTrigger"){
        std::stringstream vehicle_ids_string;
        for( auto vehicle_id : vehicle_ids ) {
            vehicle_ids_string << "_" << vehicle_id;
        }
        cpm::Logging::Instance().set_id("hlc_communicator"+vehicle_ids_string.str());
    }

void HLCCommunicator::start(){
    writeInfoMessage();
    sendReadyMessage(); 
 
    // Run this until we get a SystemTrigger to stop
    bool stop = false;
    while(!stop) {
        auto state_samples = reader_vehicleStateList.take();
        for(auto sample : state_samples) {
            // We received a StateList, which is our timing signal
            // to send commands to vehicle
            new_vehicleStateList = true;
            vehicle_state_list = sample;
        }
 
        if(new_vehicleStateList){
            runTimestep();
            new_vehicleStateList = false;
        }
         
        stop = stopSignalReceived();

        // Slow down the loop a bit
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(10)));
    }
 
    // If on_stop is defined, call it now before we finish
    if( on_stop.target_type() != typeid(void) ){ 
        on_stop();
    }
}

void HLCCommunicator::runTimestep(){
    // If this is the first timestep and the respective callback is defined, call it now
    if( first_timestep && on_first_timestep.target_type() != typeid(void) ){
        on_first_timestep(vehicle_state_list);
        first_timestep = false;
    }

    if( planning_future.valid() ){
        std::future_status future_status = planning_future.wait_for(std::chrono::milliseconds(1));
        if( future_status != std::future_status::ready ) {
            if( on_cancel_timestep.target_type() != typeid(void) ) {
                on_cancel_timestep();
            } else {
                // If we're here that means we did not manage to calculate a plan in time,
                // and we don't have a callback to stop planning early
                cpm::Logging::Instance().write(1,
                        "%s",
                        "HLC is taking too long to plan and we have no way to stop it"
                        );
                // We wait until planning has finished
                planning_future.wait();
            }
        }
        planning_future.get();
    }

    // on_each_timestep should pretty much always be defined, but we check anyway
    if( on_each_timestep.target_type() != typeid(void) ) {
        planning_future = std::async(
                on_each_timestep,
                vehicle_state_list
            );
    }
}

void HLCCommunicator::sendReadyMessage(){
    TimeStamp timestamp(11111);
    // The middleware expects a message like "hlc_${vehicle_id}", e.g. hlc_1
    for( auto vehicle_id : vehicle_ids ) {
        std::string hlc_identification("hlc_");
        hlc_identification.append(std::to_string(vehicle_id));
        ReadyStatus readyStatus(hlc_identification, timestamp);
        writer_readyStatus.write(readyStatus);
    }
}

bool HLCCommunicator::stopSignalReceived(){
    auto systemTrigger_samples = reader_systemTrigger.take();
    for (auto sample : systemTrigger_samples) {
        if (sample.next_start().nanoseconds() == trigger_stop) {
                return true;
        }
    }
    return false;
}

void HLCCommunicator::writeInfoMessage(){
    std::stringstream set_callbacks;
    std::stringstream unset_callbacks;

    if( on_first_timestep.target_type() == typeid(void)) { 
        unset_callbacks << "on_first_timestep ";
    } else {
        set_callbacks << "on_first_timestep ";
    }

    if( on_each_timestep.target_type() == typeid(void)) { 
        unset_callbacks << "on_each_timestep ";
    } else {
        set_callbacks << "on_each_timestep ";
    }

    if( on_cancel_timestep.target_type() == typeid(void)) { 
        unset_callbacks << "on_cancel_timestep ";
    } else {
        set_callbacks << "on_cancel_timestep ";
    }

    if( on_stop.target_type() == typeid(void)) { 
        unset_callbacks << "on_stop ";
    } else {
        set_callbacks << "on_stop ";
    }

    std::stringstream ss;
    ss << "Started HLCCommunicator for Vehicle ID ";
    for( auto vehicle_id : vehicle_ids ) {
        ss << static_cast<int>(vehicle_id) << " ";
    }
    ss << std::endl;
    ss << "The following callback methods were set: " << set_callbacks.rdbuf()  << std::endl;

    ss << "The following callback methods were not set: "  << unset_callbacks.rdbuf() << std::endl;

    // Write to Log as an info message and to stdout, so it appears in the text logs
    cpm::Logging::Instance().write(3, "%s", ss.str().c_str());
    std::cout << ss.str() << std::endl;
}
