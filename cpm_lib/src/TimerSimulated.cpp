#include "TimerSimulated.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/timerfd.h>
#include <unistd.h>
#include <cmath>
#define TRIGGER_STOP_SYMBOL (0xffffffffffffffffull)

TimerSimulated::TimerSimulated(
    std::string _node_id, 
    uint64_t _period_nanoseconds, 
    uint64_t _offset_nanoseconds
)
:period_nanoseconds(_period_nanoseconds)
,offset_nanoseconds(_offset_nanoseconds)
,ready_topic(cpm::ParticipantSingleton::Instance(), "ready")
,trigger_topic(cpm::ParticipantSingleton::Instance(), "system_trigger")
,writer_ready_status(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), ready_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
,reader_system_trigger(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()))
,node_id(_node_id)
,current_time(0)
{
    //Offset must be smaller than period
    if (offset_nanoseconds >= period_nanoseconds) {
        fprintf(stderr, "Offset set higher than period\n");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

    dds::sub::cond::ReadCondition read_cond(reader_system_trigger, dds::sub::status::DataState::any());
    waitset += read_cond;
}

void TimerSimulated::start(std::function<void(uint64_t t_now)> update_callback)
{
    if(this->active) {
        throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice");
    }

    this->active = true;

    m_update_callback = update_callback;
    
    uint64_t deadline = offset_nanoseconds;
    current_time = deadline;
    bool noSignalReceived = true;
    bool gotStartSignal = false;

    while(this->active) {
        //Send ready signal
        ReadyStatus ready_status;
        ready_status.next_start_stamp(TimeStamp(deadline));
        ready_status.source_id(node_id);
        if (!noSignalReceived) {
            writer_ready_status.write(ready_status); 
        }

        //Wait for any signal (in the first period only) and for the start signal
        while(!gotStartSignal) {
            //Wait for the next signals until the start signal has been received
            if (noSignalReceived) {
                writer_ready_status.write(ready_status); 
                waitset.wait(dds::core::Duration::from_millisecs(2000));
            }
            else {
                dds::core::cond::WaitSet::ConditionSeq active_conditions = waitset.wait();
            }

            for (auto sample : reader_system_trigger.take()) {
                if (sample.info().valid()) {
                    noSignalReceived = false;

                    if (sample.data().next_start().nanoseconds() == deadline) {
                        gotStartSignal = true;
                        current_time = deadline;

                        if(m_update_callback) m_update_callback(deadline);
                        deadline += period_nanoseconds;
                    }
                    else if (sample.data().next_start().nanoseconds() == TRIGGER_STOP_SYMBOL) {
                        //Received stop signal
                        this->active = false;
                        return;
                    }
                }
            }
        }
    }
}

void TimerSimulated::start_async(std::function<void(uint64_t t_now)> update_callback)
{
    if(!runner_thread.joinable())
    {
        m_update_callback = update_callback;
        runner_thread = std::thread([this](){
            this->start(m_update_callback);
        });
    }
    else
    {
        throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice");
    }
}

void TimerSimulated::stop()
{
    active = false;
    if(runner_thread.joinable())
    {
        runner_thread.join();
    }
}

TimerSimulated::~TimerSimulated()
{
    active = false;
    if(runner_thread.joinable())
    {
        runner_thread.join();
    }
}


uint64_t TimerSimulated::get_time()
{
    return current_time;
}