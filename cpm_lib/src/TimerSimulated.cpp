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

TimerSimulated::Answer TimerSimulated::handle_system_trigger(uint64_t& deadline) {
    bool got_valid = false;
    bool got_new_deadline = false;

    for (auto sample : reader_system_trigger.take()) {
        if (sample.info().valid()) {
            got_valid = true;

            if (sample.data().next_start().nanoseconds() == deadline) {
                current_time = deadline;

                if(m_update_callback) m_update_callback(deadline);
                deadline += period_nanoseconds;

                got_new_deadline = true;

                // Current period finished -> Send next ready signal
                ReadyStatus ready_status;
                ready_status.next_start_stamp(TimeStamp(deadline));
                ready_status.source_id(node_id);
                writer_ready_status.write(ready_status);
            }
            else if (sample.data().next_start().nanoseconds() == TRIGGER_STOP_SYMBOL) {
                //Received stop signal
                this->active = false;
                return Answer::STOP;
            }
        }
    }

    if (got_new_deadline) {
        return Answer::DEADLINE;
    }
    else if (got_valid) {
        return Answer::ANY;
    }
    else {
        return Answer::NONE;
    }
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

    //Send first ready signal until any signal has been received, process answer if necessary
    Answer system_trigger = Answer::NONE;
    while (system_trigger == Answer::NONE) {
        ReadyStatus ready_status;
        ready_status.next_start_stamp(TimeStamp(deadline));
        ready_status.source_id(node_id);
        writer_ready_status.write(ready_status); 
        waitset.wait(dds::core::Duration::from_millisecs(2000));

        system_trigger = handle_system_trigger(deadline);
    }

    //Wait for the next relevant time step and call the callback function until the process has been stopped
    while(this->active) {
        //Wait for the next signals until the next start signal has been received
        dds::core::cond::WaitSet::ConditionSeq active_conditions = waitset.wait();

        //Process new messages
        system_trigger = handle_system_trigger(deadline);
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