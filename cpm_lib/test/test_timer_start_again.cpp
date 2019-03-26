#include "catch.hpp"
#include "TimerFD.hpp"
#include "cpm/exceptions.hpp"
#include <unistd.h>

#include <thread>
#include <string>
#include <vector>

#include "cpm/ParticipantSingleton.hpp"
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

/**
 * Tests:
 * - Checks if the starting time, period and offset were obeyed when the timer is started asynchronously
 * - Calls start(_async) after the timer has been started -> exceptions should be thrown
 * - Checks if the timer id matches the received id
 */

TEST_CASE( "TimerFD_start_again" ) {

    std::cout << "Starting TimerFD start-again test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  0; //This time check offset = 0

    std::string timer_id = "2";
    TimerFD timer(timer_id, period, offset, true);

    //Starting time to check for:
    uint64_t starting_time = timer.get_time() + 2000000000;

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

    //Data for CHECKS / ASSERTS
    std::string source_id; //To check whether the source ID of a message is the same as the sender id
    uint64_t start_stamp; //The initial stamp sent by the timer should be zero, as stamps are only required to be used by the simulated timer (signals "I'm ready")
    std::vector<uint64_t> get_time_timestamps; //Timestamps from timer.get_time() in each call of the callback function
    std::vector<uint64_t> t_start_timestamps; //Timestamps t_now in each call of the callback function

    //Thread that handles the start signal - it receives the ready signal by the timer and sends a system trigger (start signal)
    std::thread signal_thread = std::thread([&](){
        std::cout << "TimerFD: Receiving ready signal..." << std::endl;

        //Wait for ready signal
        ReadyStatus status;
        dds::core::cond::WaitSet::ConditionSeq active_conditions = waitset.wait();
        for (auto sample : timer_ready_signal_ready.take()) {
            if (sample.info().valid()) {
                status.next_start_stamp(sample.data().next_start_stamp());
                status.source_id(sample.data().source_id());
                break;
            }
        }
        source_id = status.source_id();
        start_stamp = status.next_start_stamp().nanoseconds();

        std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

        //Send start signal
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(starting_time));
        timer_system_trigger_writer.write(trigger);
    });

    //Check if start_async works as expected as well and store timestamps
    timer.start_async([&](uint64_t t_start){
        get_time_timestamps.push_back(timer.get_time());
        t_start_timestamps.push_back(t_start);
    });

    //Check that the timer cannot be used while it is running
    usleep(1000000);
    std::cout << "Starting the timer again" << std::endl;
    CHECK_THROWS_AS(timer.start([](uint64_t t_start) {}), cpm::ErrorTimerStart);
    CHECK_THROWS_AS(timer.start_async([](uint64_t t_start) {}), cpm::ErrorTimerStart);

    //Wait for above stuff to finish
    usleep(3000000);
    if (signal_thread.joinable()) {
        signal_thread.join();
    }
    timer.stop();

    //Checks / assertions from threads
    //Check whether the ready signal is correct
    CHECK(source_id == timer_id);
    CHECK(start_stamp == 0);
    //Check if the starting time was obeyed and if period + offset were set correctly
    for (int i = 0; i < get_time_timestamps.size(); ++i) {
        CHECK( get_time_timestamps.at(i) >= starting_time + 1 ); //Was started after the initial starting time
        CHECK( t_start_timestamps.at(i) <= get_time_timestamps.at(i) ); //t_start should be smaller than the current time
        CHECK( get_time_timestamps.at(i) <= t_start_timestamps.at(i) + 1000000 ); // actual start time is within 1 ms of declared start time
        CHECK( t_start_timestamps.at(i) % period == offset ); // start time corresponds to timer definition
    }
}
