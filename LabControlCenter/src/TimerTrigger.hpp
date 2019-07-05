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
#include "dds/pub/DataWriter.hpp"
#include "dds/sub/DataReader.hpp"

#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

enum ParticipantStatus {
    WAITING, OUT_OF_SYNC, WORKING, REALTIME
};

struct TimerData {
    uint64_t next_timestep;
    uint64_t last_message_receive_stamp;
    ParticipantStatus participant_status;
};

class TimerTrigger {
private:
    const bool use_simulated_time;
    std::atomic_bool timer_running;

    //Communication objects and callbacks
    void ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples);
    dds::sub::DataReader<ReadyStatus> ready_status_reader;
    dds::pub::DataWriter<SystemTrigger> system_trigger_writer;
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
    std::atomic_bool next_signal_thread_running;
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

    std::map<string, TimerData> get_participant_message_data();

    void get_current_simulated_time(bool& use_simulated_time, uint64_t& current_time);
};