#include "TimerSimulated.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/timerfd.h>
#include <unistd.h>

TimerSimulated::TimerSimulated(
    std::string _node_id, 
    uint64_t _period_nanoseconds, 
    uint64_t _offset_nanoseconds
)
:period_nanoseconds(_period_nanoseconds)
,offset_nanoseconds(_offset_nanoseconds)
,ready_topic(cpm::ParticipantSingleton::Instance(), "ready")
,trigger_topic(cpm::ParticipantSingleton::Instance(), "system_trigger")
,writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), ready_topic)
,reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic)
,node_id(_node_id)
,period_number(0)
,current_time(0)
{
    dds::sub::cond::ReadCondition read_cond(reader, dds::sub::status::DataState::any());
    waitset += read_cond;
}

void TimerSimulated::wait() {
    uint64_t next_period;
    if (period_number == 0) {
        next_period = offset_nanoseconds;
    }
    else {
        next_period = offset_nanoseconds + period_nanoseconds;
    }
    ++period_number;

    //Send ready signal
    ReadyStatus ready_status;
    ready_status.next_start_stamp(TimeStamp(next_period));
    ready_status.source_id(node_id);
    writer.write(ready_status);
    
    //Wait for start signal
    bool gotStartSignal = false;
    while (!gotStartSignal) {
        dds::core::cond::WaitSet::ConditionSeq active_conditions =
        waitset.wait();
        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                if (sample.data().next_start().nanoseconds() == next_period) {
                    gotStartSignal = true;
                    break;
                }
                else if (sample.data().next_start().nanoseconds() == (2^64 - 1)) {
                    //Check for stop signal
                    active = false;
                }
            }
        }
    }

    //Got start signal, set new time
    current_time = next_period;
}

void TimerSimulated::start(std::function<void(uint64_t t_now)> update_callback)
{
    if(this->active) {
        std::cerr << "The cpm::Timer can not be started twice" << std::endl;
        return;
    }

    m_update_callback = update_callback;
    
    uint64_t deadline = ((this->get_time()/period_nanoseconds)+1)*period_nanoseconds + offset_nanoseconds;
    this->active = true;

    while(this->active) {
        this->wait();
        if(this->get_time() >= deadline) {
            if(m_update_callback) m_update_callback(deadline);

            deadline += period_nanoseconds;

            while(this->get_time() >= deadline)
            {
                std::cerr << "Warning, missed timestep " << deadline << std::endl;
                deadline += period_nanoseconds;
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
        std::cerr << "The cpm::Timer can not be started twice" << std::endl;
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