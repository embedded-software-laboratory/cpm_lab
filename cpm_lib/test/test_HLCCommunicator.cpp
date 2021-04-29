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

#include "catch.hpp"

#include <limits>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdlib>

#include "cpm/HLCCommunicator.hpp"
#include "cpm/ReaderAbstract.hpp"
#include "cpm/Writer.hpp"
#include "cpm/Timer.hpp"
#include "cpm/Participant.hpp"

#include "ReadyStatus.hpp"
#include "VehicleStateList.hpp"
#include "SystemTrigger.hpp"

/**
 * \test Tests HLCCommunicator
 * \ingroup cpmlib
 */
TEST_CASE( "HLCCommunicator" ) {
    cpm::Logging::Instance().set_id("test_hlc_communicator");

    int vehicle_id = 17; // Arbitrary
    int middleware_domain = 5; // Arbitrary
    uint64_t period_nanos = 47000000ull; // 47ms; arbitrary
    uint64_t period_ms = period_nanos/1000000;

    std::string local_qos_path = std::string(getenv("HOME"))+"/dev/software/cpm_lib/build/QOS_LOCAL_COMMUNICATION.xml";
    std::string local_qos_profile = "MatlabLibrary::LocalCommunicationProfile";

    cpm::Participant local_participant(
            middleware_domain,
            local_qos_path,
            local_qos_profile
            );

    cpm::Writer<VehicleStateList> writer_vehicleStateList(
            local_participant.get_participant(),
            "vehicleStateList");

    cpm::Writer<SystemTrigger> writer_systemTrigger(
            local_participant.get_participant(),
            "systemTrigger");

    cpm::ReaderAbstract<ReadyStatus> reader_readyStatus(
            local_participant.get_participant(),
            "readyStatus");

    // Create a SystemTrigger message that means stop
    uint64_t stop_trigger = std::numeric_limits<uint64_t>::max();
    TimeStamp timestamp_stop_trigger(stop_trigger);
    SystemTrigger msg_stop_trigger(timestamp_stop_trigger);

    HLCCommunicator hlc_communicator(
            vehicle_id,
            middleware_domain);

    int hlc_timestep = 0;
    std::atomic_int first_timer_timestep;
    std::atomic_int timer_timestep;
    int timestep_to_test_cancel = 4;
    int timestep_to_test_stop = 10;
    std::atomic_bool abort_timestep(false);

    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list){
            // Check that we received the right VehicleStateList
            REQUIRE( period_ms == vehicle_state_list.period_ms() );
            });

    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list){
            abort_timestep.store(false);
            hlc_timestep++;

            cpm::Logging::Instance().write(1,
                    "HLC planning timestep %i",
                    hlc_timestep);

            // Check that we didn't skip any timesteps
            REQUIRE( hlc_timestep == (timer_timestep.load() + 1 - first_timer_timestep.load()));

            // Check that we received the right VehicleStateList
            REQUIRE( period_ms == vehicle_state_list.period_ms() );

            if( hlc_timestep == timestep_to_test_cancel ) {
                cpm::Logging::Instance().write(1,
                        "%s",
                        "Testing onCancelTimestep");
                while(!abort_timestep.load()){
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                cpm::Logging::Instance().write(1,
                        "%s",
                        "Finished testing onCancelTimestep");
            }
            });

    hlc_communicator.onCancelTimestep([&]{
            abort_timestep.store(true);
            // Check that the cancel callback was called at the right timestep
            REQUIRE( hlc_timestep == timestep_to_test_cancel );
            // Check that the timer timestep is one past the hlc_timestep, so we had reason to cancel
            REQUIRE( timestep_to_test_cancel + 1 == (timer_timestep.load() + 1 - first_timer_timestep.load()));

            cpm::Logging::Instance().write(1,
                    "HLC cancelling timestep %i",
                    hlc_timestep);
            });

    hlc_communicator.onStop([&]{
            REQUIRE( hlc_timestep == timestep_to_test_stop );
            cpm::Logging::Instance().write(1,
                    "%s",
                    "HLC stopping");
            });

    // To give Reader/Writers time to connect
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::shared_ptr<cpm::Timer> mock_middleware_timer = cpm::Timer::create(
            "mock_middleware_timer",
            period_nanos,
            0,
            false,
            false,
            false);

    mock_middleware_timer->start_async([&](uint64_t t_now){
            VehicleStateList vehicle_state_list;
            vehicle_state_list.t_now(t_now);
            vehicle_state_list.period_ms(period_ms);
            writer_vehicleStateList.write(vehicle_state_list);

            timer_timestep++;

            cpm::Logging::Instance().write(1,
                    "Timer at timestep %i",
                    timer_timestep.load());

            for( auto sample : reader_readyStatus.take()){
                cpm::Logging::Instance().write(1,
                        "Mock middleware received ready message from %s",
                        sample.source_id().c_str());
                if( sample.source_id() == std::string("hlc_")+std::to_string(vehicle_id) ) {
                    first_timer_timestep.store(timer_timestep.load());
                }
            }
            });

    int timesteps_passed;
    std::future<void> future = std::async([&]{
            while(true){
                timesteps_passed = timer_timestep.load() + 1 - first_timer_timestep.load();
                if(timesteps_passed >= timestep_to_test_stop){
                    writer_systemTrigger.write(msg_stop_trigger);
                    mock_middleware_timer->stop();
                    break;
                }

                // Slow down the loop a bit
                rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(10)));
            }
            });

    hlc_communicator.start();
}
