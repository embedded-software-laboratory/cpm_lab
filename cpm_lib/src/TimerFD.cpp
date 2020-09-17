// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
        bool _wait_for_start,
        uint64_t _stop_signal
    )
    :period_nanoseconds(_period_nanoseconds)
    ,offset_nanoseconds(_offset_nanoseconds)
    ,ready_topic(cpm::get_topic<ReadyStatus>("readyStatus"))
    ,trigger_topic(cpm::get_topic<SystemTrigger>("systemTrigger"))
    ,node_id(_node_id)
    ,reader_system_trigger(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()))
    ,readCondition(reader_system_trigger, dds::sub::status::DataState::any())
    ,wait_for_start(_wait_for_start)
    ,stop_signal(_stop_signal)
    {
        //Offset must be smaller than period
        if (offset_nanoseconds >= period_nanoseconds) {
            Logging::Instance().write(
                1,
                "%s", 
                "TimerFD: Offset set higher than period."
            );
            fprintf(stderr, "Offset set higher than period.\n");
            fflush(stderr); 
            exit(EXIT_FAILURE);
        }

        //Add Waitset for reader_system_trigger
        waitset += readCondition;

        active.store(false);
    }

    void TimerFD::createTimer() {
        // Timer setup
        timer_fd = timerfd_create(CLOCK_REALTIME, 0);
        if (timer_fd == -1) {
            Logging::Instance().write(
                1,
                "%s", 
                "TimerFD: Call to timerfd_create failed."
            );
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
            Logging::Instance().write(
                1,
                "TimerFD: Call to timer_settime returned error status (%d).", 
                status
            );
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
            Logging::Instance().write(
                1,
                "TimerFD: Error: read(timerfd), status %d.", 
                status
            );
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
        
        //Poll for start signal, send ready signal every 2 seconds until the start signal has been received or the thread has been killed
        //Break if stop signal was received
        while(active.load()) {
            writer_ready_status.write(ready_status);

            waitset.wait(dds::core::Duration::from_millisecs(2000));

            for (auto sample : reader_system_trigger.take()) {
                if (sample.info().valid()) {
                    return sample.data().next_start().nanoseconds();
                }
            }
        }

        //Active is false, just return stop signal here
        return stop_signal;
    }

    void TimerFD::start(std::function<void(uint64_t t_now)> update_callback)
    {
        if(active.load()) {
            Logging::Instance().write(
            2,
            "%s", 
            "TimerFD: The cpm::Timer can not be started twice."
        );
            throw cpm::ErrorTimerStart("The cpm::Timer can not be started twice.");
        }

        active.store(true);

        m_update_callback = update_callback;

        //Create the timer (so that they operate in sync)
        createTimer();

        //Send ready signal, wait for start signal
        uint64_t deadline;
        if (wait_for_start) {
            start_point = receiveStartTime();
            
            if (start_point == stop_signal) {
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

        start_point_initialized = true;

        while(active.load()) {
            this->wait();
            if(this->get_time() >= deadline) {
                if(m_update_callback) m_update_callback(deadline);

                deadline += period_nanoseconds;

                uint64_t current_time = this->get_time();

                //Error if deadline was missed, correction to next deadline
                if (current_time >= deadline)
                {
                    Logging::Instance().write(
                        1,
                        "TimerFD: Periods missed: %d", 
                        static_cast<int>(((current_time - deadline) / period_nanoseconds) + 1)
                    );

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
                        active.store(false);
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
            Logging::Instance().write(
                2,
                "%s", 
                "TimerFD: The cpm::Timer can not be started twice."
            );
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
        active.store(false);
        if(runner_thread.joinable())
        {
            runner_thread.join();
        }
    }

    TimerFD::~TimerFD()
    {
        active.store(false);
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

    uint64_t TimerFD::get_start_time()
    {
        //Return 0 if not yet started or stopped before started
        if (!start_point_initialized) return 0;

        return start_point;
    }

    bool TimerFD::received_stop_signal() 
    {
        dds::sub::LoanedSamples<SystemTrigger> samples = reader_system_trigger.take();

        for (auto sample : samples) 
        {
            if(sample.info().valid())
            {
                if (sample.data().next_start().nanoseconds() == stop_signal) 
                {
                    return true;
                }
            }
        }

        return false;
    }

}