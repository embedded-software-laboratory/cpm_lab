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
#include "cpm/TimerFD.hpp"
#include <unistd.h>

#include <thread>

#include "cpm/ParticipantSingleton.hpp"
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

/**
 * Tests:
 * - Sends a stop signal but not a start signal
 * - Therefore: Makes sure that the timer callback function is never actually called
 */

TEST_CASE( "TimerFD_stop_signal" ) {
    //Set the Logger ID
    cpm::Logging::Instance().set_id("test_timerfd_stop_signal");

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    std::string timer_id = "0";
    cpm::TimerFD timer(timer_id, period, offset, true);

    //Writer to send system triggers to the timer 
    dds::pub::DataWriter<SystemTrigger> writer_SystemTrigger(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),          
        dds::topic::find<dds::topic::Topic<SystemTrigger>>(cpm::ParticipantSingleton::Instance(), "systemTrigger"), 
        (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));
    //Reader to receive ready signals from the timer
    dds::sub::DataReader<ReadyStatus> reader_ReadyStatus(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
        dds::topic::find<dds::topic::Topic<ReadyStatus>>(cpm::ParticipantSingleton::Instance(), "readyStatus"), 
        (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));
    
    //Waitset to wait for any data
    dds::core::cond::WaitSet waitset;
    dds::sub::cond::ReadCondition read_cond(reader_ReadyStatus, dds::sub::status::DataState::any());
    waitset += read_cond;


    //Thread to send a stop signal after the ready signal was received
    std::thread signal_thread = std::thread([&](){

        //Wait for ready signal
        ReadyStatus status;
        waitset.wait();
        for (auto sample : reader_ReadyStatus.take()) {
            if (sample.info().valid()) {
                break;
            }
        }

        //Send stop signal
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(cpm::TRIGGER_STOP_SYMBOL));
        writer_SystemTrigger.write(trigger);
    });

    //Ignore warning that t_start is unused
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"

    timer.start([&](uint64_t t_start){
        //The timer should never start because it is stopped before that can happen (No start signal is sent)
        CHECK(false);
    });

    //This part can only be reached if the timer was stopped

    #pragma GCC diagnostic pop

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

}
