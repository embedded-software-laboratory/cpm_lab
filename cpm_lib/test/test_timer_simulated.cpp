#include "catch.hpp"
#include "TimerSimulated.hpp"
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
        int64_t period_diff = diff_1 - diff_2;
        CHECK(((period_diff >= - 1000000) && (period_diff <= 1000000))); //Ready signal sent periodically (within 1ms)
        std::cout << period_diff << std::endl;

        for (int i = 0; i <= num_runs; ++i) {
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
            CHECK(status.source_id() == "1");
            CHECK(status.next_start_stamp().nanoseconds() == period * i + offset);

            std::cout << "TimerFD: Received ready signal: " << status.source_id() << " " << status.next_start_stamp() << std::endl;

            next_start = status.next_start_stamp().nanoseconds();

            if (i != 0) {
                std::cout << "The timer should not start before 'X' is printed" << std::endl;
            }

            //Send wrong start signal
            SystemTrigger trigger;
            trigger.next_start(TimeStamp(next_start - 2));
            writer.write(trigger);

            //Send wrong start signal
            trigger.next_start(TimeStamp(next_start - 1));
            writer.write(trigger);

            usleep(100000);
            if (i != 0) {
                std::cout << "X" << std::endl;
            }
            else {
                //Wait for 5 seconds, no new ready signal should be received
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);
                uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
                waitset.wait(dds::core::Duration(5, 0));
                clock_gettime(CLOCK_REALTIME, &t);
                uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

                CHECK(t2 - t1 >= 5000000000);
            }

            //Send correct start signal
            trigger.next_start(TimeStamp(next_start));
            writer.write(trigger);
        }

        //Send stop signal
        active_conditions = waitset.wait();

        std::cout << "Sending stop signal..." << std::endl;
        uint64_t two = 2;
        uint64_t max_time = two^63 - 1;
        max_time += two ^63;

        SystemTrigger stop_trigger;
        stop_trigger.next_start(TimeStamp(max_time));
        writer.write(stop_trigger);
    });

    timer.start([&](uint64_t t_start){
        std::cout << "Next time step" << std::endl;

        uint64_t now = timer.get_time();

        CHECK( t_start == now + period );
        CHECK( (now - offset) % period == 0);
        CHECK( now == count * period + offset );

        count++;

        t_start_prev = t_start;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime (not really useful here)
    });

    //Should be obsolete because the timer was already stopped before; but: required to free resources
    timer.stop();

    if (signal_thread.joinable()) {
        signal_thread.join();
    }
}
