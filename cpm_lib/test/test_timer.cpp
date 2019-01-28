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

TEST_CASE( "TimerFD_accuracy" ) {

    std::cout << "Starting TimerFD test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    TimerFD timer("0", period, offset);

    int count = 0;
    uint64_t t_start_prev = 0;

    //Starting time to check for:
    uint64_t starting_time = timer.get_time() + 1000000;

    //Reader / Writer for ready status and system trigger
    dds::pub::DataWriter<SystemTrigger> writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),          
        dds::topic::find<dds::topic::Topic<SystemTrigger>>(cpm::ParticipantSingleton::Instance(), "system_trigger"), 
        (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));
    dds::sub::DataReader<ReadyStatus> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
        dds::topic::find<dds::topic::Topic<ReadyStatus>>(cpm::ParticipantSingleton::Instance(), "ready"), 
        (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));

    //Thread for start signal
    std::thread signal_thread = std::thread([&](){
        //Waitset to wait for data
        // Create a WaitSet
        dds::core::cond::WaitSet waitset;
        // Create a ReadCondition for a reader with a specific DataState
        dds::sub::cond::ReadCondition read_cond(
            reader, dds::sub::status::DataState::any());
        // Attach conditions
        waitset += read_cond;

        std::cout << "TimerFD: Receiving ready signal..." << std::endl;

        //Wait for ready signal
        ReadyStatus status;
        dds::core::cond::WaitSet::ConditionSeq active_conditions =
            waitset.wait();
        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                status.next_start_stamp(sample.data().next_start_stamp());
                status.source_id(sample.data().source_id());
                break;
            }
        }
        CHECK(status.source_id() == "0");
        CHECK(status.next_start_stamp().nanoseconds() == 0);

        std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

        //Send start signal
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(starting_time));
        writer.write(trigger);
    });

    timer.start([&](uint64_t t_start){
        uint64_t now = timer.get_time();

        CHECK( now >= starting_time );
        CHECK( t_start <= now );
        CHECK( now <= t_start + 1000000 ); // actual start time is within 1 ms of declared start time
        CHECK( t_start % period == offset ); // start time corresponds to timer definition

        if(count > 0)
        {
            CHECK( ((count%3)+1)*period == t_start - t_start_prev);
        }

        count++;
        if(count > 15) {
            timer.stop();
        }

        t_start_prev = t_start;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }
}
