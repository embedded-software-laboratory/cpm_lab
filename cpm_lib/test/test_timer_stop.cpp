#include "catch.hpp"
#include "TimerFD.hpp"
#include <unistd.h>

#include <thread>

#include "cpm/ParticipantSingleton.hpp"
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#define TRIGGER_STOP_SYMBOL (0xffffffffffffffffull)

/**
 * Tests:
 * - Sends a stop signal but not a start signal
 * - Therefore: Makes sure that the timer callback function is never actually called
 */

TEST_CASE( "TimerFD_stop_signal" ) {

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    std::string timer_id = "0";
    TimerFD timer(timer_id, period, offset, true);

    //Writer to send system triggers to the timer 
    dds::pub::DataWriter<SystemTrigger> timer_system_trigger_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),          
        dds::topic::find<dds::topic::Topic<SystemTrigger>>(cpm::ParticipantSingleton::Instance(), "system_trigger"), 
        (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));
    //Reader to receive ready signals from the timer
    dds::sub::DataReader<ReadyStatus> timer_ready_signal_ready(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
        dds::topic::find<dds::topic::Topic<ReadyStatus>>(cpm::ParticipantSingleton::Instance(), "ready"), 
        (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));
    
    //Waitset to wait for any data
    dds::core::cond::WaitSet waitset;
    dds::sub::cond::ReadCondition read_cond(timer_ready_signal_ready, dds::sub::status::DataState::any());
    waitset += read_cond;


    //Thread to send a stop signal after the ready signal was received
    std::thread signal_thread = std::thread([&](){
        std::cout << "TimerFD: Receiving ready signal..." << std::endl;

        //Wait for ready signal
        ReadyStatus status;
        waitset.wait();
        for (auto sample : timer_ready_signal_ready.take()) {
            if (sample.info().valid()) {
                break;
            }
        }

        //Send stop signal
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(TRIGGER_STOP_SYMBOL));
        timer_system_trigger_writer.write(trigger);
    });

    timer.start([&](uint64_t t_start){
        //The timer should never start because it is stopped before that can happen (No start signal is sent)
        CHECK(false);
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

}
