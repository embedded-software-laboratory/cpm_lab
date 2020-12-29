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
     * This class calls a callback function periodically 
     * based on a simulated clock. The calls are synchronized in both frequency 
     * and phase to the clock. Time is not 'real', but given by a central timing instance (e.g. the LCC)
     * \ingroup cpmlib
     */
    class TimerSimulated : public cpm::Timer
    {
        uint64_t period_nanoseconds; 
        uint64_t offset_nanoseconds;
        cpm::Writer<ReadyStatus> writer_ready_status;
        dds::sub::DataReader<SystemTrigger> reader_system_trigger;
        std::string node_id;
        uint64_t current_time;

        std::atomic_bool active;
        std::atomic_bool cancelled; //In rare cases, stop can be called even before active is set to true; in that case, active alone does not suffice
        std::mutex join_mutex; //join does not work concurrently

        std::thread runner_thread;
        std::function<void(uint64_t t_now)> m_update_callback;
        std::function<void()> m_stop_callback;

        dds::core::cond::WaitSet waitset;

        enum Answer {STOP, DEADLINE, ANY, NONE};
        /**
         * \brief Takes all received messages (since the last function call) from reader_system_trigger. If new messages could be received, checks whether they are significant (stop signal or signal matching the current deadline). If so, react accordingly: Stop the system when a stop signal has been received, refresh the current time and the deadline and call the callback function if the new deadline has been reached + send a new ready signal afterwards.
         * \param deadline Current deadline of the system / the next time step when the system should be activated
         */
        Answer handle_system_trigger(uint64_t& deadline);

    public:
        TimerSimulated(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds);
        ~TimerSimulated();

        void start       (std::function<void(uint64_t t_now)> update_callback) override;
        void start       (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;
        void start_async (std::function<void(uint64_t t_now)> update_callback) override;
        void start_async (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;
        void stop() override;
        uint64_t get_time() override;
        uint64_t get_start_time() override;
    };

}