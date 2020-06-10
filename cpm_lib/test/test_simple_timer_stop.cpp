#include "catch.hpp"
#include "cpm/SimpleTimer.hpp"
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
 * - Sends a custom stop signal and checks whether it works
 * - Therefore: Makes sure that the timer callback function is never actually called
 */

TEST_CASE( "SimpleTimer_custom_stop_signal" ) {
    //Set the Logger ID
    cpm::Logging::Instance().set_id("test_simple_timer_custom_stop_signal");

    const uint64_t period_ms = 100;
    bool wait_for_start = true;
    bool react_to_stop = true;
    uint64_t custom_stop_signal = 1234;
    std::string timer_id = "0";
    cpm::SimpleTimer timer(timer_id, period_ms, wait_for_start, react_to_stop, custom_stop_signal);

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
        trigger.next_start(TimeStamp(custom_stop_signal));
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
