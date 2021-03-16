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

#include "cpm/exceptions.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"

#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

namespace cpm {

    /**
     * \class TimerSimulated
     * \brief This class calls a callback function periodically 
     * based on a simulated clock. The calls are synchronized in both frequency 
     * and phase to the clock. Time is not 'real', but given by a central timing instance (e.g. the LCC)
     * \ingroup cpmlib
     */
    class TimerSimulated : public cpm::Timer
    {
    private: 
        //! Periodicity with which the timer calls a callback function
        uint64_t period_nanoseconds; 
        //! Offset from the common starting time 0 of all timers from which the periodic behaviour should start
        uint64_t offset_nanoseconds;
        //! Writer for ready status, telling the network that the timer exists and that it finished its current load and is ready for its next time step
        cpm::Writer<ReadyStatus> writer_ready_status;
        //! Used to receive start, stop and intermediate timing signals
        dds::sub::DataReader<SystemTrigger> reader_system_trigger;
        //! ID of the timer, e.g. middleware, e.g. for identification in the timer tab of the LCC
        std::string node_id;
        //! Current simulated time, also used by get_time
        uint64_t current_time;

        //! Timer is (in)active
        std::atomic_bool active;
        //! In rare cases, stop can be called even before active is set to true; in that case, active alone does not suffice
        std::atomic_bool cancelled; 
        //! Join does not work concurrently, but the timer stop function might be used by different threads
        std::mutex join_mutex; 

        //! For async start / running
        std::thread runner_thread;
        //! Callback function that the timer is supposed to call periodically
        std::function<void(uint64_t t_now)> m_update_callback;
        //! Optional function for when a stop signal is received
        std::function<void()> m_stop_callback;

        //! To set the waiting time for reading data (system trigger)
        dds::core::cond::WaitSet waitset;

        /**
         * \enum Answer
         * \brief Determines type of received answer
         */
        enum Answer {STOP, DEADLINE, ANY, NONE};

        /**
         * \brief Takes all received messages (since the last function call) from reader_system_trigger. If new messages could be received, checks whether they are significant (stop signal or signal matching the current deadline). If so, react accordingly: Stop the system when a stop signal has been received, refresh the current time and the deadline and call the callback function if the new deadline has been reached + send a new ready signal afterwards.
         * \param deadline Current deadline of the system / the next time step when the system should be activated
         */
        Answer handle_system_trigger(uint64_t& deadline);

    public:
        /**
         * \brief Constructor
         * \param _node_id ID of the timer in the network
         * \param period_nanoseconds The timer is called periodically with a period of period_nanoseconds
         * \param offset_nanoseconds Initial offset (from timestamp 0)
         */
        TimerSimulated(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds);

        /**
         * \brief Destructor
         */
        ~TimerSimulated();

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