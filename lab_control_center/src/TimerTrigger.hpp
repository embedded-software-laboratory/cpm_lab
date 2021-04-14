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

/**
 * \enum ParticipantStatus
 * \brief Possible status of a participant
 * \ingroup lcc
 */
enum ParticipantStatus {
    WAITING, OUT_OF_SYNC, WORKING, REALTIME
};

/**
 * \struct TimerData
 * \brief Data definition for timing related data
 * \ingroup lcc
 */
struct TimerData {
    //! For simulated time: Next point in time when the participant should receive a start signal
    uint64_t next_timestep;
    //! Timestamp for the last received message
    uint64_t last_message_receive_stamp;
    //! Status of the participant
    ParticipantStatus participant_status;
};

/**
 * \brief TimerTrigger is responsible for:
 * 
 * - Realtime: Sending start & stop signals to start / stop the simulation
 * 
 * - Simulated time: Managing timing for all participants with simulated time steps
 * 
 * See also: https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Timer+-+Periodicity+and+Synchronization
 * \ingroup lcc
 */
class TimerTrigger {
private:
    //! To determine if simulated or real time should be used
    const bool use_simulated_time;
    //! To determine if the timer is running
    std::atomic_bool timer_running;

    //Communication objects and callbacks
    /**
     * \brief Get ReadyStatus messages and new start requests, if they are newer than the current time (mostly used for simulated time).
     * In real-time, this is only relevant for the initial ready messages, which are also displayed in the UI to see which timer is ready.
     */

    bool obtain_new_ready_signals();

    /**
     * \brief TODO
     */
    bool obtain_new_stop_request_signals();

    /**
     * \brief TODO
     * \param samples
     */
    void stop_request_callback(std::vector<StopRequest>& samples);
    //! DDS Reader to obtain ReadyStatus messages sent within the network
    dds::sub::DataReader<ReadyStatus> ready_status_reader;
    //! TODO
    cpm::AsyncReader<StopRequest> stop_request_reader;
    //! DDS Writer to send SystemTrigger messages, with which timers in the network can be started / controlled (simulated time) / stopped
    cpm::Writer<SystemTrigger> system_trigger_writer;
    //! Always stores the highest timestamp that was sent by each participant
    std::map<string, TimerData> ready_status_storage;
    //! Mutex for accessing ready_status_storage
    std::mutex ready_status_storage_mutex;
    //! Stores current time as simulated time. Only makes sense if simulated time is used.
    uint64_t current_simulated_time;
    //! Mutex for accessing the current simulated time
    std::mutex simulated_time_mutex;

    //Timing functions
    //! Timer thread that handles receiving + sending timing messages in a more ordered fashion
    std::thread next_signal_thread;
    //! When sending the start signal: Decides if simulated or real time is used, cannot be reset afterwards
    std::atomic_bool simulation_started;
    /**
     * \brief Send time signals if simulated time is used, check received messages (which participant is currently working, which is out of sync etc.)
     * \returns true if a next trigger signal was sent (progress to next simulated time step), else false
     */
    bool check_signals_and_send_next_signal();

public:
    /**
     * \brief Constructor for the timer trigger, which also determines if it should be used as a real-time or simulated trigger
     * \param simulated_time Determines if the trigger should be used in a real-time or simulated-time scenario
     */
    TimerTrigger(bool simulated_time);
    /**
     * \brief Deconstructor, destroys the timing thread
     */
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
     * \brief Retrieve participants that:
     * 
     * - Realtime: Sent a message to indicate to the user of the LCC that they are now online (data is visualized in UI)
     * 
     * - Simulated time: Registered for use of simulated time
     */
    std::map<string, TimerData> get_participant_message_data();

    /**
     * \brief Obtain the current simulated time
     * \param use_simulated_time Return value: If simulated time is used by the TimerTrigger object
     * \param current_time Return value: Current simulated time (has no meaning if real time is used)
     */
    void get_current_simulated_time(bool& use_simulated_time, uint64_t& current_time);
};
