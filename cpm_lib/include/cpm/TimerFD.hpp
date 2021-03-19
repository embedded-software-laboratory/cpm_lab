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

#include <mutex>
#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

#include "cpm/exceptions.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/Writer.hpp"

#include <atomic>

namespace cpm {
    /**
     * \class TimerFD
     * \brief This class calls a callback function periodically 
     * based the system clock. The calls are synchronized in both frequency 
     * and phase to the clock.
     * This class should be used if a 'simple' timed callback is required. 
     * Whereas Timer.hpp relies on e.g. a parameter server to check if 
     * simulated time is used or not, directly using TimerFD allows to create
     * timed callbacks for methods that use the system clock independent on 
     * the run (real or simulated). This could be GUI tools, periodic tasks etc... 
     * \ingroup cpmlib
     */
    class TimerFD : public cpm::Timer
    {
    private:
    
        //! Periodicity with which the timer calls a callback function
        uint64_t period_nanoseconds; 
        //! Offset from the 1970 epoch where the counting starts, should usually stay the same value 0 for all used timers in the network
        uint64_t offset_nanoseconds;
        //! ID of the timer, e.g. middleware, e.g. for identification in the timer tab of the LCC
        std::string node_id;

        //Cannot be substituted by other cpm classes and was not abstracted
        //Usage here: Wait for data on 'take' up to x ms or until the read condition is fulfilled
        //! Used to receive start and stop signals
        dds::sub::DataReader<SystemTrigger> reader_system_trigger;
        //! Read condition to wait for data
        dds::sub::cond::ReadCondition readCondition;
        //! To set the waiting time for the read condition
        dds::core::cond::WaitSet waitset;

        //! Writer for ready status, telling the network that the timer exists and is ready to operate
        cpm::Writer<ReadyStatus> writer_ready_status;
        
        //! Timer is (in)active
        std::atomic_bool active;
        //! In rare cases, stop can be called even before active is set to true; in that case, active alone does not suffice
        std::atomic_bool cancelled;
        //! Join does not work concurrently, but the timer stop function might be used by different threads
        std::mutex join_mutex;
        //! Internal timer, set up with the according unix functions in the constructor
        int timer_fd = -1;
        //! For async start / running
        std::thread runner_thread;
        //! Callback function that the timer is supposed to call periodically
        std::function<void(uint64_t t_now)> m_update_callback;
        //! Optional function for when a stop signal is received
        std::function<void()> m_stop_callback;

        /**
         * \brief Wait for the next period start of timerfd
         */
        void wait();

        /**
         * \brief Wait for a start signal; 
         * return the start signal as soon as one was received
         * return the stop signal if that one was received
         */
        uint64_t receiveStartTime();
        //! The start signal tells the timer when to start (timestamp in ns), which is stored here
        uint64_t start_point = 0;
        //! Only retrieve start_point from outside if it was initialized (mutex costs too much performance, is only written once)
        bool start_point_initialized = false;
        
        /**
         * \brief True if a stop signal has been received
         */
        bool received_stop_signal ();
        
        //! If false, do not use receiveStartTime()
        const bool wait_for_start;

        /**
         * \brief Create the internal timerf using the realtime clock
         */
        void createTimer ();

        //! For custom stop signals, should be changed only if you know what you are doing (usually you do not want to define a stop signal for you own participant, but use the default one!)
        uint64_t stop_signal = TRIGGER_STOP_SYMBOL;

    public:
        /**
         * \brief Create a "real-time" timer that can be used for function callback
         * \param _node_id ID of the timer in the network
         * \param period_nanoseconds The timer is called periodically with a period of period_nanoseconds
         * \param offset_nanoseconds Initial offset (from timestamp 0)
         * \param wait_for_start Set whether the timer is started only if a start signal is sent via DDS (true), or if it should should start immediately (false)
         * \param _stop_signal Optional and not recommended unless you know what you are doing! Define your own stop signal (instead of the default one) for DDS communication
         */
        TimerFD(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool wait_for_start, uint64_t _stop_signal = TRIGGER_STOP_SYMBOL);
        
        /**
         * \brief Destructor for internal mutex, timerfd...
         */
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

        /**
         * \brief Can be used to obtain the time the timer was started in nanoseconds
         * \return The start time of the timer, either received as start signal or from internal start, in nanoseconds OR
         * 0 if not yet started or stopped before started
         */
        uint64_t get_start_time() override;
    };

}