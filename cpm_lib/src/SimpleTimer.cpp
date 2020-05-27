#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "cpm/SimpleTimer.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include "cpm/get_topic.hpp"

namespace cpm {

    SimpleTimer::SimpleTimer(
        std::string _node_id, 
        uint64_t _period_milliseconds, 
        bool _wait_for_start
    )
    :period_milliseconds(_period_milliseconds)
    ,ready_topic(cpm::get_topic<ReadyStatus>("readyStatus"))
    ,trigger_topic(cpm::get_topic<SystemTrigger>("systemTrigger"))
    ,node_id(_node_id)
    ,reader_system_trigger(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()))
    ,readCondition(reader_system_trigger, dds::sub::status::DataState::any())
    ,wait_for_start(_wait_for_start)
    {
        //Add Waitset for reader_system_trigger
        waitset += readCondition;
    }

    bool SimpleTimer::wait_until(const std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>& timeout_time)
    {
        std::unique_lock<std::mutex> lock(condition_mutex);
        return abort_condition.wait_until(lock, timeout_time, [&]{ return active; });
    }

    uint64_t SimpleTimer::receiveStartTime() {
        //Reader / Writer for ready status and system trigger
        dds::pub::DataWriter<ReadyStatus> writer_ready_status(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), ready_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));

        //Create ready signal
        ReadyStatus ready_status;
        ready_status.next_start_stamp(TimeStamp(0));
        ready_status.source_id(node_id);
        
        //Poll for start signal, send ready signal every 2 seconds until the start signal has been received
        //Break if stop signal was received
        while(true) {
            writer_ready_status.write(ready_status);

            waitset.wait(dds::core::Duration::from_millisecs(2000));

            for (auto sample : reader_system_trigger.take()) {
                if (sample.info().valid()) {
                    return sample.data().next_start().nanoseconds();
                }
            }
        }
    }

    void SimpleTimer::start(std::function<void(uint64_t t_now)> update_callback)
    {
        if(this->active) {
            Logging::Instance().write("%s", "SimpleTimer: The cpm::Timer can not be started twice.");
            throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice.");
        }

        {
            std::lock_guard<std::mutex> lock(condition_mutex);
            this->active = true;
        }

        m_update_callback = update_callback;

        //Send ready signal, wait for start signal
        uint64_t start_point;
        uint64_t deadline;
        if (wait_for_start) {
            start_point = receiveStartTime();
            
            if (start_point == TRIGGER_STOP_SYMBOL) {
                return;
            }
        }
        else {
            start_point = this->get_time();
        }

        auto next_start = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>(
            std::chrono::nanoseconds(start_point)
        );

        while(this->active) {
            if (!this->wait_until(next_start))
            {
                //We already stopped the timer in between waiting
                break;
            }

            //This check is only required for the time before the start_point, to make sure that the timer does not start too early
            if(this->get_time() >= start_point) {
                if(m_update_callback) m_update_callback(this->get_time());

                if (received_stop_signal()) {
                    //Either stop the timer or call the stop callback function, if one exists
                    if (m_stop_callback)
                    {
                        m_stop_callback();
                    }
                    else 
                    {
                        std::lock_guard<std::mutex> lock(condition_mutex);
                        this->active = false;
                    }
                }
            }

            //Update next start time slot until it is greater than the current time
            while(next_start < std::chrono::steady_clock::now() && active)
            {
                next_start += std::chrono::milliseconds(period_milliseconds);
            }
        }
    }

    void SimpleTimer::start(std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback)
    {
        m_stop_callback = stop_callback;
        start(update_callback);
    }

    void SimpleTimer::start_async(std::function<void(uint64_t t_now)> update_callback)
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
            Logging::Instance().write("%s", "SimpleTimer: The cpm::Timer can not be started twice.");
            throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice.");
        }
    }

    void SimpleTimer::start_async(std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) 
    {
        m_stop_callback = stop_callback;
        start_async(update_callback);
    }

    void SimpleTimer::stop()
    {
        std::lock_guard<std::mutex> lock(condition_mutex);
        active = false;
        abort_condition.notify_all();

        if(runner_thread.joinable())
        {
            runner_thread.join();
        }
    }

    SimpleTimer::~SimpleTimer()
    {
        std::lock_guard<std::mutex> lock(condition_mutex);
        active = false;
        abort_condition.notify_all();

        if(runner_thread.joinable())
        {
            runner_thread.join();
        }
    }


    uint64_t SimpleTimer::get_time()
    {
        return cpm::get_time_ns();
    }

    bool SimpleTimer::received_stop_signal() 
    {
        dds::sub::LoanedSamples<SystemTrigger> samples = reader_system_trigger.take();

        for (auto sample : samples) 
        {
            if(sample.info().valid())
            {
                if (sample.data().next_start().nanoseconds() == TRIGGER_STOP_SYMBOL) 
                {
                    return true;
                }
            }
        }

        return false;
    }

}