#include "TimerSimulated.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/timerfd.h>
#include <unistd.h>
#include <cmath>

TimerSimulated::TimerSimulated(
    std::string _node_id, 
    uint64_t _period_nanoseconds, 
    uint64_t _offset_nanoseconds
)
:period_nanoseconds(_period_nanoseconds)
,offset_nanoseconds(_offset_nanoseconds)
,ready_topic(cpm::ParticipantSingleton::Instance(), "ready")
,trigger_topic(cpm::ParticipantSingleton::Instance(), "system_trigger")
,writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), ready_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
,reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()))
,node_id(_node_id)
,period_number(0)
,current_time(0)
{
    //Offset must be smaller than period
    if (offset_nanoseconds >= period_nanoseconds) {
        offset_nanoseconds = period_nanoseconds - 1;
        std::cerr << "Offset set higher than period" << std::endl;
    }

    dds::sub::cond::ReadCondition read_cond(reader, dds::sub::status::DataState::any());
    waitset += read_cond;
}

void TimerSimulated::wait() {
    uint64_t next_period = 0;
    bool noSignalReceived = false;
    bool gotStartSignal = false;

    if (period_number == 0) {
        next_period += offset_nanoseconds;
        noSignalReceived = true;
    }
    else {
        next_period += offset_nanoseconds + period_number * period_nanoseconds;
    }

    //Send ready signal
    ReadyStatus ready_status;
    ready_status.next_start_stamp(TimeStamp(next_period));
    ready_status.source_id(node_id);
    if (!noSignalReceived) {
        writer.write(ready_status); //Soll das ready Signal stattdessen nach jeder Abfrage des Servers geschrieben werden?
    }

    //Wait for any signal (in the first period only) and for the start signal
    do {
        //Send the first ready signal until any signal has been received (only in the first period)
        if (noSignalReceived) {
            writer.write(ready_status);
            rti::util::sleep(dds::core::Duration(2));
        }
        else { //Wait for the next signals until the start signal has been received
            dds::core::cond::WaitSet::ConditionSeq active_conditions = waitset.wait();
        }

        uint64_t two = 2;
        uint64_t max_time = pow(two, 63) - 1;

        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                noSignalReceived = false;
                if (sample.data().next_start().nanoseconds() == next_period) {
                    gotStartSignal = true;
                    break;
                }
                else if (sample.data().next_start().nanoseconds() == max_time) {
                    //Received stop signal
                    gotStartSignal = true;
                    this->active = false;
                    break;
                }
            }
        }
    }
    while(!gotStartSignal);

    ++period_number;

    //Got start signal, set new time
    current_time = next_period;
}

void TimerSimulated::start(std::function<void(uint64_t t_now)> update_callback)
{
    if(this->active) {
        std::cerr << "The cpm::Timer can not be started twice" << std::endl;
        return;
    }

    this->active = true;

    m_update_callback = update_callback;
    
    uint64_t deadline = this->get_time() + offset_nanoseconds;

    while(this->active) {
        this->wait();
        if(this->get_time() >= deadline) { //Kann eigentlich hier entfernt werden, wait kümmert sich schon darum
            if(m_update_callback) m_update_callback(deadline);

            deadline += period_nanoseconds;

            while(this->get_time() >= deadline)
            {
                std::cerr << "Warning, missed timestep " << deadline << std::endl;
                deadline += period_nanoseconds;
            }
        }
    }

    std::cout << "Timer stopped" << std::endl;
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