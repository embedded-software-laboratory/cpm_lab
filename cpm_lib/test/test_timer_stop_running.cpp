#include "catch.hpp"
#include "TimerFD.hpp"
#include <unistd.h>

#include <thread>
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
 * - Tests if the timer can be stopped by sending a stop signal
 * - Also checks if the timer id matches the received id
 */

TEST_CASE( "TimerFD_stop_signal_when_running" ) {

    std::cout << "Starting TimerFD test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
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

    //Variables for CHECKs
    std::string source_id; //To check whether the received message was really sent from the timer
    uint64_t start_stamp; //The start stamp should be 0 as a real time timer is used
    int count = 0; //The thread should be stopped before it is called three times

    //Thread to receive the ready signal, send a start signal and then a stop signal after 100ms
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

        //Wait
        rti::util::sleep(dds::core::Duration::from_millisecs(100));

        //Send stop signal
        trigger.next_start(TimeStamp(TRIGGER_STOP_SYMBOL));
        timer_system_trigger_writer.write(trigger);
    });

    //Callback function of the timer
    timer.start([&](uint64_t t_start){
        CHECK(count <= 2); //This task should not be called too often
        usleep( 100000 ); // simluate variable runtime
        ++count;
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

    //Check that the ready signal matches the expected ready signal
    CHECK(source_id == "0");
    CHECK(start_stamp == 0);
}
