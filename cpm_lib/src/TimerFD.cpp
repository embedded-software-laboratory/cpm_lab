#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "cpm/TimerFD.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/timerfd.h>
#include <unistd.h>
#include <stdint.h>
#include "cpm/get_topic.hpp"

namespace cpm {

    TimerFD::TimerFD(
        std::string _node_id, 
        uint64_t _period_nanoseconds, 
        uint64_t _offset_nanoseconds,
        bool _wait_for_start
    )
    :period_nanoseconds(_period_nanoseconds)
    ,offset_nanoseconds(_offset_nanoseconds)
    ,ready_topic(cpm::get_topic<ReadyStatus>("readyStatus"))
    ,trigger_topic(cpm::get_topic<SystemTrigger>("system_trigger"))
    ,node_id(_node_id)
    ,reader_system_trigger(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()))
    ,readCondition(reader_system_trigger, dds::sub::status::DataState::any())
    ,wait_for_start(_wait_for_start)
    {
        //Offset must be smaller than period
        if (offset_nanoseconds >= period_nanoseconds) {
            Logging::Instance().write("TimerFD: Offset set higher than period.");
            fprintf(stderr, "Offset set higher than period.\n");
            fflush(stderr); 
            exit(EXIT_FAILURE);
        }

        //Add Waitset for reader_system_trigger
        waitset += readCondition;
    }

    void TimerFD::createTimer() {
        // Timer setup
        timer_fd = timerfd_create(CLOCK_REALTIME, 0);
        if (timer_fd == -1) {
            Logging::Instance().write("TimerFD: Call to timerfd_create failed.");
            fprintf(stderr, "Call to timerfd_create failed.\n"); 
            perror("timerfd_create");
            fflush(stderr); 
            exit(EXIT_FAILURE);
        }

        uint64_t offset_nanoseconds_fd = offset_nanoseconds;

        if(offset_nanoseconds_fd == 0) { // A zero value disarms the timer, overwrite with a negligible 1 ns.
            offset_nanoseconds_fd = 1;
        }

        struct itimerspec its;
        its.it_value.tv_sec     = offset_nanoseconds_fd / 1000000000ull;
        its.it_value.tv_nsec    = offset_nanoseconds_fd % 1000000000ull;
        its.it_interval.tv_sec  = period_nanoseconds / 1000000000ull;
        its.it_interval.tv_nsec = period_nanoseconds % 1000000000ull;
        int status = timerfd_settime(timer_fd, TFD_TIMER_ABSTIME, &its, NULL);
        if (status != 0) {
            Logging::Instance().write("TimerFD: Call to timer_settime returned error status (%d).", status);
            fprintf(stderr, "Call to timer_settime returned error status (%d).\n", status);
            fflush(stderr); 
            exit(EXIT_FAILURE);
        }
    }

    void TimerFD::wait()
    {
        unsigned long long missed;
        int status = read(timer_fd, &missed, sizeof(missed));
        if(status != sizeof(missed)) {
            Logging::Instance().write("TimerFD: Error: read(timerfd), status %d.", status);
            fprintf(stderr, "Error: read(timerfd), status %d.\n", status);
            fflush(stderr); 
            exit(EXIT_FAILURE);
        }
    }

    uint64_t TimerFD::receiveStartTime() {
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

    void TimerFD::start(std::function<void(uint64_t t_now)> update_callback)
    {
        if(this->active) {
            Logging::Instance().write("TimerFD: The cpm::Timer can not be started twice.");
            throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice.");
        }

        this->active = true;

        m_update_callback = update_callback;

        //Create the timer (so that they operate in sync)
        createTimer();

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

        if ((start_point - offset_nanoseconds) % period_nanoseconds == 0) {
            deadline = start_point;
        }
        else {
            deadline = (((start_point - offset_nanoseconds) / period_nanoseconds) + 1) * period_nanoseconds + offset_nanoseconds;
        }

        while(this->active) {
            this->wait();
            if(this->get_time() >= deadline) {
                if(m_update_callback) m_update_callback(deadline);

                deadline += period_nanoseconds;

                uint64_t current_time = this->get_time();

                //Error if deadline was missed, correction to next deadline
                if (current_time >= deadline)
                {
                    Logging::Instance().write("TimerFD: Deadline: %" PRIu64 ", current time: %" PRIu64 ", periods missed: %" PRIu64, deadline, current_time, ((current_time - deadline) / period_nanoseconds) + 1);

                    deadline += (((current_time - deadline)/period_nanoseconds) + 1)*period_nanoseconds;
                }

                if (received_stop_signal()) {
                    //Either stop the timer or call the stop callback function, if one exists
                    if (m_stop_callback)
                    {
                        m_stop_callback();
                    }
                    else 
                    {
                        this->active = false;
                    }
                }
            }
        }

        close(timer_fd);
    }

    void TimerFD::start(std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback)
    {
        m_stop_callback = stop_callback;
        start(update_callback);
    }

    void TimerFD::start_async(std::function<void(uint64_t t_now)> update_callback)
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
            Logging::Instance().write("TimerFD: The cpm::Timer can not be started twice.");
            throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice.");
        }
    }

    void TimerFD::start_async(std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) 
    {
        m_stop_callback = stop_callback;
        start_async(update_callback);
    }

    void TimerFD::stop()
    {
        active = false;
        if(runner_thread.joinable())
        {
            runner_thread.join();
        }
    }

    TimerFD::~TimerFD()
    {
        active = false;
        if(runner_thread.joinable())
        {
            runner_thread.join();
        }
        close(timer_fd);
    }


    uint64_t TimerFD::get_time()
    {
        return cpm::get_time_ns();
    }

    bool TimerFD::received_stop_signal() 
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