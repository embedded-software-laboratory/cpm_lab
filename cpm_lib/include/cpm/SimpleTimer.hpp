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

#include "cpm/TimerFD.hpp"

#include <cmath>

namespace cpm {
    /**
     * \class SimpleTimer
     * This class calls a callback function periodically 
     * based on TimerFD. The given period (in milliseconds) is broken down in intervals of 50ms, to be able
     * to stop the timer rather quickly when stop() is called. The given period in milliseconds might thus be rounded up accordingly. 
     * This timer is neither intended to work with simulated time,
     * nor is it exact enough to be real-time capable. Use this e.g. for timing in the GUI or other non-critical
     * timing tasks only!
     * This timer listens to the stop signal if m_stop_callback is not set
     */
    class SimpleTimer : public cpm::Timer
    {
        std::shared_ptr<cpm::TimerFD> internal_timer;
        //The internal timer works with 50ms as interval, but a counter is used to call the actual callback function only every period_milliseconds (rounded up to 50ms)
        uint64_t internal_timer_counter = 0;
        uint64_t counter_max = 0;
        uint64_t fifty_ms = 50000000ull;

        std::function<void(uint64_t t_now)> m_update_callback;
        std::function<void()> m_stop_callback;

        void simple_timer_callback(uint64_t t_now);

    public:
        /**
         * \brief Create a simple timer (not real-time capable) that can be used for function callback
         * \param node_id ID of the timer in the network
         * \param period_milliseconds The timer is called periodically with a period of period_milliseconds (rounded up to 50ms)
         * \param wait_for_start Set whether the timer is started only if a start signal is sent via DDS (true), or if it should should start immediately (false)
         * \param react_to_stop_signal Set whether the timer should be stopped if a stop signal is sent within the network (optional, default is true); 
         * \param stop_signal Set your own custom stop signal - unrecommended, unless you know exactly what you are doing
         * if set, stop_callback is set to be an empty lambda if start is called without stop_callback
         */
        SimpleTimer(std::string _node_id, uint64_t period_milliseconds, bool wait_for_start, bool react_to_stop_signal = true, uint64_t stop_signal = TRIGGER_STOP_SYMBOL);
        ~SimpleTimer();

        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called.
         * \param update_callback the callback function, which in this case just gets the current time
         */
        void start       (std::function<void(uint64_t t_now)> update_callback) override;

        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called. When a stop signal is received, the stop_callback function is called (and may also call stop(), if desired).
         * This allows to user to define a custom stop behaviour, e.g. that the vehicle that uses the timer only stops driving,
         * but does not stop the internal timer.
         * \param update_callback the callback function to call when the next timestep is reached, which in this case just gets the current time
         * \param stop_callback the callback function to call when the timer is stopped
         */
        void start       (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;

        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * \param update_callback the callback function, which in this case just gets the current time
         */
        void start_async (std::function<void(uint64_t t_now)> update_callback) override;

        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * When a stop signal is received, the stop_callback function is called (and may also call stop(), if desired).
         * This allows to user to define a custom stop behaviour, e.g. that the vehicle that uses the timer only stops driving,
         * but does not stop the internal timer.
         * \param update_callback the callback function to call when the next timestep is reached, which in this case just gets the current time
         * \param stop_callback the callback function to call when the timer is stopped
         */
        void start_async (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;

        /**
         * \brief Stops the periodic callback and kills the thread (if it was created using start_async).
         */
        void stop() override;

        /**
         * \brief Can be used to obtain the current system time in milliseconds.
         * \return the current system time in milliseconds
         */
        uint64_t get_time() override;

        /**
         * \brief Can be used to obtain the time the timer was started in nanoseconds
         * \return The start time of the timer, either received as start signal or from internal start, in nanoseconds OR
         * 0 if not yet started or stopped before started
         */
        uint64_t get_start_time() override;
    };

}