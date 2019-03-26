#include "catch.hpp"
#include "TimerSimulated.hpp"
#include "TimerFD.hpp"
#include <unistd.h>
#include <cmath>

#include <thread>
#include <vector>
#include <string>

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
 * - Does the source ID match the sender ID (data remains unchanged, no other data was received during the test)
 * - Do the ready signals match the expectation (offset and period are correct, increase by period in each step)
 * - Are start signals that do not match the ready signal ignored
 * - Is the current time stamp correct (regarding offset and period)
 * - Does the thread time match the current time (the simulated timestamp should be the same as t_now)
 */

TEST_CASE( "TimerSimulated_accuracy" ) {

    std::cout << "Starting TimerFD (simulated) test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    std::string timer_id = "1";
    TimerSimulated timer(timer_id, period, offset);

    int count = 0; //Count how often the timer callback was called
    int num_runs = 15; //Run the timer for 15 periods

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
    dds::sub::cond::ReadCondition read_cond(
        timer_ready_signal_ready, dds::sub::status::DataState::any());
    waitset += read_cond;

    //Assertion / check data for both threads, to be checked after the execution
    std::vector<ReadyStatus> status_ready; //All ready status signals received from the timer in each run
    std::vector<std::vector<ReadyStatus>> status_wrong_start_signal; //All ready status signals received from the timer after a wrong start signal was sent
    std::vector<uint64_t> get_time_timestamps; //Timestamps from timer.get_time() in each call of the callback function
    std::vector<uint64_t> t_start_timestamps; //Timestamps t_now in each call of the callback function

    //Thread that handles the simulated time - it receives ready signals by the timer and sends system triggers
    std::thread signal_thread = std::thread([&](){
        uint64_t next_start; 

        for (int i = 0; i < num_runs; ++i) {
            //Wait for ready signal
            ReadyStatus status;
            waitset.wait();
            for (auto sample : timer_ready_signal_ready.take()) {
                if (sample.info().valid()) {
                    status.next_start_stamp(sample.data().next_start_stamp());
                    status.source_id(sample.data().source_id());
                    break;
                }
            }
            status_ready.push_back(status);

            std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

            next_start = status.next_start_stamp().nanoseconds();

            //Send wrong start signal
            SystemTrigger trigger;
            trigger.next_start(TimeStamp(next_start - 2));
            timer_system_trigger_writer.write(trigger);

            //Send wrong start signal
            trigger.next_start(TimeStamp(next_start - 1));
            timer_system_trigger_writer.write(trigger);

            //Wait for up to 2 seconds, no new ready signal should be received
            waitset.wait(dds::core::Duration(2, 0)); //Wait either for 2 seconds or for a new signal
            std::vector<ReadyStatus> signals;
            for (auto sample : timer_ready_signal_ready.take()) {
                if (sample.info().valid()) {
                    signals.push_back(sample.data());
                }
            }
            status_wrong_start_signal.push_back(signals);

            //Send correct start signal
            trigger.next_start(TimeStamp(next_start));
            timer_system_trigger_writer.write(trigger);
        }

        //Send stop signal - after num_runs, the callback function should not be called again
        waitset.wait();

        std::cout << "Sending stop signal..." << std::endl;

        SystemTrigger stop_trigger;
        stop_trigger.next_start(TimeStamp(TRIGGER_STOP_SYMBOL));
        timer_system_trigger_writer.write(stop_trigger);
    });

    //Registering callback function for the timer - Just simulate a variable runtime and save the timestamps in each run as well as the number of runs
    timer.start([&](uint64_t t_start){
        std::cout << "Next time step" << std::endl;

        get_time_timestamps.push_back(timer.get_time());
        t_start_timestamps.push_back(t_start);

        count++;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime (not really useful here)
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

    //Checks and assertions
    for (int i = 0; i < status_ready.size(); ++i) {
        CHECK(status_ready.at(i).source_id() == timer_id); //The source id should always match the id set for the timer
        CHECK(status_ready.at(i).next_start_stamp().nanoseconds() == period * i + offset); //The ready stamps should match the settings for period and offset
    }
    for (int i = 0; i < status_wrong_start_signal.size(); ++i) {
        for (auto signal : status_wrong_start_signal.at(i)) {
            CHECK(signal.next_start_stamp().nanoseconds() == period * i + offset); //After "wrong" start signals have been sent, the ready signal should remain unchanged
        }
    }
    for (int i = 0; i < get_time_timestamps.size(); ++i) {
        //Check if the thread times match the current time + if the current timestamp is correct
        CHECK( t_start_timestamps.at(i) == get_time_timestamps.at(i) );
        CHECK( (get_time_timestamps.at(i) - offset) % period == 0);
        CHECK( t_start_timestamps.at(i) == i * period + offset );
    }
    //No more than num_runs runs should have taken place
    REQUIRE(count == num_runs);
}
