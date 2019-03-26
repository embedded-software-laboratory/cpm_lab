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

/**
 * Tests:
 * - Is the timer started after the initial starting time
 * - Does t_now match the expectation regarding offset, period and start values
 * - Is the callback function called shortly after t_now
 * - Is the timer actually stopped when it should be stopped
 * - If the callback function takes longer than period to finish, is this handled correctly
 */

TEST_CASE( "TimerFD_accuracy" ) {

    std::cout << "Starting TimerFD test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;

    //Variables for the callback function
    int count = 0;
    uint64_t t_start_prev = 0;
    bool was_stopped = false;

    TimerFD timer("0", period, offset, true);

    //Starting time to check for:
    uint64_t starting_time = timer.get_time() + 3000000000;

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

    //Variables for CHECKs - only to identify the timer by its id and to check if the start stamp was correct
    std::string source_id;
    uint64_t start_stamp;

    //Thread to receive the ready signal and send a start signal afterwards
    std::thread signal_thread = std::thread([&](){
        std::cout << "TimerFD: Receiving ready signal..." << std::endl;

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
        source_id = status.source_id();
        start_stamp = status.next_start_stamp().nanoseconds();

        std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

        //Send start signal
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(starting_time));
        timer_system_trigger_writer.write(trigger);
    });

    timer.start([&](uint64_t t_start){
        uint64_t now = timer.get_time();

        CHECK( was_stopped == false ); //Should never be called if it was stopped
        CHECK( now >= starting_time + period * count); //Curent timer should match the expectation regarding starting time and period
        if (count == 0) {
            CHECK( t_start <= starting_time + period + 1000000); // actual start time is within 1 ms of initial start time
        }
        CHECK( t_start <= now ); //Callback should not be called before t_now
        CHECK( now <= t_start + 1000000 ); // actual start time is within 1 ms of declared start time
        CHECK( t_start % period == offset ); // start time corresponds to timer definition

        if(count > 0)
        {
            CHECK( ((count%3)+1)*period == t_start - t_start_prev); //Fitting to the sleep behaviour, the difference between the periods should match this expression
        }

        count++;
        if(count > 15) {
            timer.stop();
            was_stopped = true;
        }

        t_start_prev = t_start;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime that can be greater than period
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

    //Check that the ready signal matches the expected ready signal
    CHECK(source_id == "0");
    CHECK(start_stamp == 0);
}
