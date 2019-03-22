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

TEST_CASE( "TimerSimulated_accuracy" ) {

    std::cout << "Starting TimerFD (simulated) test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    TimerSimulated timer("1", period, offset);

    int count = 0;
    int num_runs = 15;
    uint64_t t_start_prev = 0;

    //Reader / Writer for ready status and system trigger
    dds::pub::DataWriter<SystemTrigger> writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),          
        dds::topic::find<dds::topic::Topic<SystemTrigger>>(cpm::ParticipantSingleton::Instance(), "system_trigger"), 
        (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));
    dds::sub::DataReader<ReadyStatus> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
        dds::topic::find<dds::topic::Topic<ReadyStatus>>(cpm::ParticipantSingleton::Instance(), "ready"), 
        (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));
    //Waitset to wait for data
    // Create a WaitSet
    dds::core::cond::WaitSet waitset;
    // Create a ReadCondition for a reader with a specific DataState
    dds::sub::cond::ReadCondition read_cond(
        reader, dds::sub::status::DataState::any());
    // Attach conditions
    waitset += read_cond;

    //Assertion / check data for both threads, to be checked after the execution
    int64_t period_diff;
    std::vector<ReadyStatus> status_ready;
    std::vector<std::vector<ReadyStatus>> status_wrong_start_signal;
    std::vector<uint64_t> get_time_timestamps;
    std::vector<uint64_t> t_start_timestamps;

    //Thread for start signal
    std::thread signal_thread = std::thread([&](){
        uint64_t next_start; 

        //Check if ready signal is sent periodically
        std::cout << "TimerFD: Receiving ready signal..." << std::endl;
        dds::core::cond::WaitSet::ConditionSeq active_conditions = waitset.wait();
        uint64_t time_1 = timer.get_time();
        active_conditions = waitset.wait();
        uint64_t time_2 = timer.get_time();
        active_conditions = waitset.wait();
        uint64_t time_3 = timer.get_time();

        uint64_t diff_1 = time_2 - time_1;
        uint64_t diff_2 = time_3 - time_2;
        period_diff = diff_1 - diff_2;

        for (int i = 0; i < num_runs; ++i) {
            //Wait for ready signal
            ReadyStatus status;
            active_conditions = waitset.wait();
            for (auto sample : reader.take()) {
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
            writer.write(trigger);

            //Send wrong start signal
            trigger.next_start(TimeStamp(next_start - 1));
            writer.write(trigger);

            //Wait for up to 2 seconds, no new ready signal should be received
            waitset.wait(dds::core::Duration(2, 0)); //Wait either for 2 seconds or for a new signal
            std::vector<ReadyStatus> signals;
            for (auto sample : reader.take()) {
                if (sample.info().valid()) {
                    signals.push_back(sample.data());
                }
            }
            status_wrong_start_signal.push_back(signals);

            //Send correct start signal
            trigger.next_start(TimeStamp(next_start));
            writer.write(trigger);
        }

        //Send stop signal
        active_conditions = waitset.wait();

        std::cout << "Sending stop signal..." << std::endl;

        SystemTrigger stop_trigger;
        stop_trigger.next_start(TimeStamp(TRIGGER_STOP_SYMBOL));
        writer.write(stop_trigger);
    });

    timer.start([&](uint64_t t_start){
        std::cout << "Next time step" << std::endl;

        get_time_timestamps.push_back(timer.get_time());
        t_start_timestamps.push_back(t_start);

        count++;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime (not really useful here)
    });

    //Should be obsolete because the timer was already stopped before; but: required to free resources
    timer.stop();

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

    //Checks and assertions
    CHECK(((period_diff >= - 1000000) && (period_diff <= 1000000))); //Ready signal sent periodically (within 1ms)
    for (int i = 0; i < status_ready.size(); ++i) {
        CHECK(status_ready.at(i).source_id() == "1"); //The source id should always match the id set for the timer
        CHECK(status_ready.at(i).next_start_stamp().nanoseconds() == period * i + offset); //The ready stamps should match the settings for period and offset
    }
    for (int i = 0; i < status_wrong_start_signal.size(); ++i) {
        for (auto signal : status_wrong_start_signal.at(i)) {
            CHECK(signal.next_start_stamp().nanoseconds() == period * i + offset); //After "wrong" start signals have been sent, the ready signal should remain unchanged
        }
    }
    for (int i = 0; i < get_time_timestamps.size(); ++i) {
        CHECK( t_start_timestamps.at(i) == get_time_timestamps.at(i) );
        CHECK( (get_time_timestamps.at(i) - offset) % period == 0);
        CHECK( t_start_timestamps.at(i) == i * period + offset );
    }
}
