#include "catch.hpp"
#include "TimerSimulated.hpp"
#include "cpm/TimerFD.hpp"
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

/**
 * Tests:
 * - Does the source ID match the sender ID (data remains unchanged, no other data was received during the test)
 * - Do the ready signals match the expectation (offset and period are correct, increase by period in each step)
 * - Are start signals that do not match the ready signal ignored
 * - Is the current time stamp correct (regarding offset and period)
 * - Does the thread time match the current time (the simulated timestamp should be the same as t_now)
 */

TEST_CASE( "TimerSimulated_accuracy" ) {
    //Set the Logger ID
    cpm::Logging::Instance().set_id("test_timer_simulated_accuracy");

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    std::string timer_id = "qwertzy";
    cpm::TimerSimulated timer(timer_id, period, offset);

    const int num_runs = 5;

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
    dds::sub::cond::ReadCondition read_cond(
        reader_ReadyStatus, dds::sub::status::DataState::any());
    waitset += read_cond;


    /** Test result values **/

    // All ready status signals received from the timer in each run
    std::vector<ReadyStatus> status_ready;

    // Timestamps from timer.get_time() in each call of the callback function
    std::vector<uint64_t> get_time_timestamps;

    // Timestamps t_now in each call of the callback function 
    std::vector<uint64_t> t_start_timestamps; 



    // Thread that handles the simulated time - it receives 
    // ready signals by the timer and sends system triggers
    std::thread signal_thread = std::thread([&](){

        for (int i = 0; i < num_runs; ++i) {
            // Wait for ready signal
            ReadyStatus status;
            waitset.wait();
            for (auto sample : reader_ReadyStatus.take()) {
                if (sample.info().valid()) {
                    status = sample.data();
                    break;
                }
            }
            status_ready.push_back(status);
            
            uint64_t next_start = status.next_start_stamp().nanoseconds();

            //Send wrong start signal
            SystemTrigger trigger;
            trigger.next_start(TimeStamp(next_start - 2));
            writer_SystemTrigger.write(trigger);

            //Send wrong start signal
            trigger.next_start(TimeStamp(next_start - 1));
            writer_SystemTrigger.write(trigger);

            //Wait, no new ready signal should be received
            waitset.wait(dds::core::Duration(0, 500000000));

            //Ignore warning that sample is not used
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wunused-but-set-variable"

            for (auto sample : reader_ReadyStatus.take()) {
                //No new sample should be received -> We never want to enter this loop
                CHECK(false);
            }

            #pragma GCC diagnostic pop

            //Send correct start signal
            trigger.next_start(TimeStamp(next_start));
            writer_SystemTrigger.write(trigger);
        }

        //Send stop signal - after num_runs, the callback function should not be called again
        waitset.wait();

        SystemTrigger stop_trigger;
        stop_trigger.next_start(TimeStamp(cpm::TRIGGER_STOP_SYMBOL));
        writer_SystemTrigger.write(stop_trigger);
    });


    int timer_loop_count = 0; //timer_loop_count how often the timer callback was called

    // save the timestamps in each run as well as the number of runs
    timer.start([&](uint64_t t_start){
        get_time_timestamps.push_back(timer.get_time());
        t_start_timestamps.push_back(t_start);

        timer_loop_count++;
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

    // Checks
    for (size_t i = 0; i < status_ready.size(); ++i) {
        //The source id should always match the id set for the timer
        CHECK(status_ready.at(i).source_id() == timer_id); 
        //The ready stamps should match the settings for period and offset
        CHECK(status_ready.at(i).next_start_stamp().nanoseconds() == period * i + offset); 
    }

    for (size_t i = 0; i < get_time_timestamps.size(); ++i) {
        //Check if the thread times match the current time and if the current timestamp is correct
        CHECK( t_start_timestamps.at(i) == get_time_timestamps.at(i) );
        CHECK( (get_time_timestamps.at(i) - offset) % period == 0);
        CHECK( t_start_timestamps.at(i) == i * period + offset );
    }

    //No more than num_runs runs should have taken place
    REQUIRE(timer_loop_count == num_runs);
}
