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

TEST_CASE( "TimerFD_stop_signal" ) {

    std::cout << "Starting TimerFD test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;

    TimerFD timer("0", period, offset, true);

    //Reader / Writer for ready status and system trigger
    dds::pub::DataWriter<SystemTrigger> writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),          
        dds::topic::find<dds::topic::Topic<SystemTrigger>>(cpm::ParticipantSingleton::Instance(), "system_trigger"), 
        (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));
    dds::sub::DataReader<ReadyStatus> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
        dds::topic::find<dds::topic::Topic<ReadyStatus>>(cpm::ParticipantSingleton::Instance(), "ready"), 
        (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));
    // Create a WaitSet
    dds::core::cond::WaitSet waitset;
    // Create a ReadCondition for a reader with a specific DataState
    dds::sub::cond::ReadCondition read_cond(
        reader, dds::sub::status::DataState::any());
    // Attach conditions
    waitset += read_cond;

    //Variables for CHECKs
    int64_t period_diff;
    std::string source_id;
    uint64_t start_stamp;

    //Thread for start signal
    std::thread signal_thread = std::thread([&](){
        std::cout << "TimerFD: Receiving ready signal..." << std::endl;

        //Check if ready signal is sent periodically
        dds::core::cond::WaitSet::ConditionSeq active_conditions = waitset.wait();
        uint64_t time_1 = timer.get_time();
        active_conditions = waitset.wait();
        uint64_t time_2 = timer.get_time();
        active_conditions = waitset.wait();
        uint64_t time_3 = timer.get_time();

        uint64_t diff_1 = time_2 - time_1;
        uint64_t diff_2 = time_3 - time_2;
        period_diff = diff_1 - diff_2;


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
        source_id = status.source_id();
        start_stamp = status.next_start_stamp().nanoseconds();

        std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

        //Send stop signal
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(TRIGGER_STOP_SYMBOL));
        writer.write(trigger);
    });

    timer.start([&](uint64_t t_start){
        //The timer should never start because it is stopped before that can happen (No start signal is sent)
        CHECK(false);
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }

    //CHECKs of the thread
    CHECK(((period_diff >= - 1000000) && (period_diff <= 1000000))); //Ready signal sent periodically (within 1ms)
    //Check that the ready signal matches the expected ready signal
    CHECK(source_id == "0");
    CHECK(start_stamp == 0);
}
