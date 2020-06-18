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

#pragma once

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

#include "cpm/exceptions.hpp"
#include "cpm/get_time_ns.hpp"

/**
 * \class TimerFD.hpp
 * This class calls a callback function periodically 
 * based the system clock. The calls are synchronized in both frequency 
 * and phase to the clock.
 * This class should be used if a 'simple' timed callback is required. 
 * Whereas Timer.hpp relies on e.g. a parameter server to check if 
 * simulated time is used or not, directly using TimerFD allows to create
 * timed callbacks for methods that use the system clock independent on 
 * the run (real or simulated). This could be GUI tools, periodic tasks etc... 
 */

namespace cpm {

    class TimerFD : public cpm::Timer
    {
        uint64_t period_nanoseconds; 
        uint64_t offset_nanoseconds;
        dds::topic::Topic<ReadyStatus> ready_topic;
        dds::topic::Topic<SystemTrigger> trigger_topic;
        std::string node_id;
        dds::sub::DataReader<SystemTrigger> reader_system_trigger;

        dds::sub::cond::ReadCondition readCondition;
        dds::core::cond::WaitSet waitset;
        
        bool active = false;
        int timer_fd = -1;
        std::thread runner_thread;
        std::function<void(uint64_t t_now)> m_update_callback;
        std::function<void()> m_stop_callback;


        void wait();
        uint64_t receiveStartTime(); //Bool: true if start signal was received, false if stop signal was received
        bool received_stop_signal ();
        
        const bool wait_for_start; //If false, do not use receiveStartTime()

        void createTimer ();

        //For custom stop signals, should be changed only if you know what you are doing (usually you do not want to define a stop signal for you own participant, but use the default one!)
        uint64_t stop_signal = TRIGGER_STOP_SYMBOL;

    public:
        /**
         * \brief Create a "real-time" timer that can be used for function callback
         * \param node_id ID of the timer in the network
         * \param period_nanoseconds The timer is called periodically with a period of period_nanoseconds
         * \param offset_nanoseconds Initial offset (from timestamp 0)
         * \param wait_for_start Set whether the timer is started only if a start signal is sent via DDS (true), or if it should should start immediately (false)
         * \param _stop_signal Optional and not recommended unless you know what you are doing! Define your own stop signal (instead of the default one) for DDS communication
         */
        TimerFD(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool wait_for_start, uint64_t _stop_signal = TRIGGER_STOP_SYMBOL);
        ~TimerFD();

        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called.
         * \param update_callback the callback function
         */
        void start       (std::function<void(uint64_t t_now)> update_callback) override;

        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called. When a stop signal is received, the stop_callback function is called (and may also call stop(), if desired).
         * This allows to user to define a custom stop behaviour, e.g. that the vehicle that uses the timer only stops driving,
         * but does not stop the internal timer.
         * \param update_callback the callback function to call when the next timestep is reached
         * \param stop_callback the callback function to call when the timer is stopped
         */
        void start       (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;

        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * \param update_callback the callback function
         */
        void start_async (std::function<void(uint64_t t_now)> update_callback) override;

        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * When a stop signal is received, the stop_callback function is called (and may also call stop(), if desired).
         * This allows to user to define a custom stop behaviour, e.g. that the vehicle that uses the timer only stops driving,
         * but does not stop the internal timer.
         * \param update_callback the callback function to call when the next timestep is reached
         * \param stop_callback the callback function to call when the timer is stopped
         */
        void start_async (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;

        /**
         * \brief Stops the periodic callback and kills the thread (if it was created using start_async).
         */
        void stop() override;

        /**
         * \brief Can be used to obtain the current system time in nanoseconds.
         * \return the current system time in nanoseconds
         */
        uint64_t get_time() override;
    };

}