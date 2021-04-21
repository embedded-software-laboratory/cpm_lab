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

#include "MiddlewareListener.hpp"

MiddlewareListener::MiddlewareListener(int _vehicle_id, int middleware_domain, std::string qos_file, std::string qos_profile):
    vehicle_id(_vehicle_id),
    local_comms_participant(
            middleware_domain,
            qos_file,
            qos_profile
    ),
    writer_readyStatus(
            local_comms_participant.get_participant(),
            "readyStatus",
            true,
            true,
            true),
    writer_stopRequest(
            local_comms_participant.get_participant(),
            "stopRequest"),
    reader_vehicleStateList(
            local_comms_participant.get_participant(),
            "vehicleStateList"),
    reader_systemTrigger(
            local_comms_participant.get_participant(),
            "systemTrigger"){
        cpm::Logging::Instance().set_id("middleware_listener_"+std::to_string(vehicle_id));
    }

void MiddlewareListener::start(){
    //TODO: Write a short message, with "Starting now with these callbacks set, and these callbacks unset"
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
            vehicleStateList = sample;
        }
 
        if(new_vehicleStateList){
            runTimestep();
            new_vehicleStateList = false;
        }
         
        stop = stopSignalReceived();
 
        // Rate-limit this loop; usually we only get a VehicleStateList every 100-400ms
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
 
    // If onStop is defined, call it now before we finish
    if( onStop.target_type() != typeid(void) ){ 
        onStop();
    }
}

void MiddlewareListener::runTimestep(){
    // If this is the first timestep and the respective callback is defined, call it now
    if( first_timestep && onFirstTimestep.target_type() != typeid(void) ){
        onFirstTimestep(vehicleStateList);
        first_timestep = false;
    }

    if( planning_future.valid() ){
        std::future_status future_status = planning_future.wait_for(std::chrono::milliseconds(1));
        if( future_status != std::future_status::ready ) {
            if( onCancelTimestep.target_type() != typeid(void) ) {
                onCancelTimestep();
            } else {
                // If we're here that means we did not manage to calculate a plan in time,
                // and we don't have a callback to stop planning early
                cpm::Logging::Instance().write(1,
                        "HLC is taking too long to plan and we have no way to stop it"
                        );
            }
        }
        planning_future.get();
    }

    // onEachTimestep should pretty much always be defined, but we check anyway
    if( onEachTimestep.target_type() != typeid(void) ) {
        planning_future = std::async(
                onEachTimestep,
                vehicleStateList
            );
    }
}

void MiddlewareListener::sendReadyMessage(){
    TimeStamp timestamp(11111);
    // The middleware expects a message like "hlc_${vehicle_id}", e.g. hlc_1
    std::string hlc_identification("hlc_");
    hlc_identification.append(std::to_string(vehicle_id));
    ReadyStatus readyStatus(hlc_identification, timestamp);
    writer_readyStatus.write(readyStatus);
}

bool MiddlewareListener::stopSignalReceived(){
    auto systemTrigger_samples = reader_systemTrigger.take();
    for (auto sample : systemTrigger_samples) {
        if (sample.next_start().nanoseconds() == trigger_stop) {
                return true;
        }
    }
    return false;
}

void MiddlewareListener::writeInfoMessage(){
    std::stringstream set_callbacks;
    std::stringstream unset_callbacks;

    if( onFirstTimestep.target_type() == typeid(void)) { 
        unset_callbacks << "onFirstTimestep ";
    } else {
        set_callbacks << "onFirstTimestep ";
    }

    if( onEachTimestep.target_type() == typeid(void)) { 
        unset_callbacks << "onEachTimestep ";
    } else {
        set_callbacks << "onEachTimestep ";
    }

    if( onCancelTimestep.target_type() == typeid(void)) { 
        unset_callbacks << "onCancelTimestep ";
    } else {
        set_callbacks << "onCancelTimestep ";
    }

    if( onStop.target_type() == typeid(void)) { 
        unset_callbacks << "onStop ";
    } else {
        set_callbacks << "onStop ";
    }

    std::stringstream ss;
    ss << "Started MiddlewareListener for Vehicle ID " << vehicle_id  << std::endl;
    ss << "The following callback methods were set: " << set_callbacks.rdbuf()  << std::endl;

    ss << "The following callback methods were not set: "  << unset_callbacks.rdbuf() << std::endl;

    // Write to Log as an info message and to stdout, so it appears in the text logs
    cpm::Logging::Instance().write(3, ss.str().c_str());
    std::cout << ss.str() << std::endl;
}
