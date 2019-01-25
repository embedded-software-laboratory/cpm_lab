#include "catch.hpp"
#include "TimerSimulated.hpp"
#include <unistd.h>

#include <thread>

#include "cpm/ParticipantSingleton.hpp"
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

TEST_CASE( "TimerSimulated_accuracy" ) {

    std::cout << "Starting TimerFD test" << std::endl;

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    TimerSimulated timer("1", period, offset);

    int count = 0;
    int num_runs = 15;
    uint64_t t_start_prev = 0;

    //Reader / Writer for ready status and system trigger
    dds::pub::DataWriter<SystemTrigger> writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), dds::topic::find<dds::topic::Topic<SystemTrigger>>(cpm::ParticipantSingleton::Instance(), "system_trigger"));
    dds::sub::DataReader<ReadyStatus> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), dds::topic::find<dds::topic::Topic<ReadyStatus>>(cpm::ParticipantSingleton::Instance(), "ready"));
    //Waitset to wait for data
    // Create a WaitSet
    dds::core::cond::WaitSet waitset;
    // Create a ReadCondition for a reader with a specific DataState
    dds::sub::cond::ReadCondition read_cond(
        reader, dds::sub::status::DataState::any());
    // Attach conditions
    waitset += read_cond;

    //Thread for start signal
    std::thread signal_thread = std::thread([&](){
        uint64_t next_start; 

        for (int i = 0; i <= num_runs; ++i) {
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
            CHECK(status.source_id() == "1");
            CHECK(status.next_start_stamp().nanoseconds() == period * i + offset);

            std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

            next_start = status.next_start_stamp().nanoseconds();

            //Send wrong start signal
            SystemTrigger trigger;
            trigger.next_start(TimeStamp(next_start - 1));
            writer.write(trigger);

            //Send correct start signal
            trigger.next_start(TimeStamp(next_start));
            writer.write(trigger);
        }
    });

    timer.start([&](uint64_t t_start){
        uint64_t now = timer.get_time();

        CHECK( t_start == now );
        CHECK( (now - offset) % period == 0);

        count++;
        if(count >= num_runs) {
            timer.stop();
        }

        t_start_prev = t_start;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime
    });

    if (signal_thread.joinable()) {
        signal_thread.join();
    }
}
