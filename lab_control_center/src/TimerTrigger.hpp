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

#include "defaults.hpp"
#include <cassert>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/Writer.hpp"
#include "cpm/AsyncReader.hpp"
#include "dds/sub/DataReader.hpp"

#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"
#include "StopRequest.hpp"

enum ParticipantStatus {
    WAITING, OUT_OF_SYNC, WORKING, REALTIME
};

struct TimerData {
    uint64_t next_timestep;
    uint64_t last_message_receive_stamp;
    ParticipantStatus participant_status;
};

/**
 * \brief TimerTrigger is responsible for:
 * - Realtime: Sending start & stop signals to start / stop the simulation
 * - Simulated time: Managing timing for all participants with simulated time steps
 */
class TimerTrigger {
private:
    const bool use_simulated_time;
    std::atomic_bool timer_running;

    //Communication objects and callbacks
    bool obtain_new_ready_signals();
    bool obtain_new_stop_request_signals();
    void stop_request_callback(std::vector<StopRequest>& samples);
    dds::sub::DataReader<ReadyStatus> ready_status_reader;
    cpm::Writer<SystemTrigger> system_trigger_writer;
    cpm::AsyncReader<StopRequest> stop_request_reader;
    std::map<string, TimerData> ready_status_storage; //Always stores the highest timestamp that was sent by each participant
    std::mutex ready_status_storage_mutex;
    uint64_t current_simulated_time; //Only makes sense if simulated time is used
    std::mutex simulated_time_mutex;

    //Timing functions
    /**
     * \brief Send time signals if simulated time is used, check received messages
     * \returns true if a signal was sent, else false
     */
    std::thread next_signal_thread;
    std::atomic_bool simulation_started;
    bool check_signals_and_send_next_signal();

public:
    TimerTrigger(bool simulated_time);
    ~TimerTrigger();

    //Timing functions
    /**
     * \brief Send a start signal
     */
    void send_start_signal();
    /**
     * \brief Send a stop signal once
     */
    void send_stop_signal();

    /**
     * \brief Retrieve participants that
     * - Realtime: Sent a message to indicate to the user of the LCC that they are now online (data is visualized in UI)
     * - Simulated time: Registered for use of simulated time
     */
    std::map<string, TimerData> get_participant_message_data();

    void get_current_simulated_time(bool& use_simulated_time, uint64_t& current_time);
};
