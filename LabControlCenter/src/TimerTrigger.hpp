#pragma once

#include "defaults.hpp"
#include <cassert>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

struct TimerData {
    uint64_t next_timestep;
    uint64_t last_message_receive_stamp;
    std::string waiting_for_response;
};

enum WaitingResponse {
    YES, OUT_OF_SYNC, NO
};

class TimerTrigger {
private:
    bool use_simulated_time = false;
    std::atomic_bool timer_running;

    //Communication objects and callbacks
    void ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples);
    cpm::AsyncReader<ReadyStatus> ready_status_reader;
    dds::pub::DataWriter<SystemTrigger> system_trigger_writer;
    std::map<string, TimerData> ready_status_storage; //Always stores the highest timestamp that was sent by each participant
    std::mutex ready_status_storage_mutex;
    uint64_t current_simulated_time; //Only makes sense if simulated time is used
    std::mutex simulated_time_mutex;

    //Timing functions
    /**
     * \brief Send time signals if simulated time is used
     * \returns true if a signal was sent, else false
     */
    bool send_next_signal();

    //Get the current time in ns
    uint64_t get_current_time_ns();

public:
    TimerTrigger(bool simulated_time);

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

    /**
     * \brief Get the time diff to the current time as string in (minutes:)seconds (minutes if seconds > 60)
     */
    std::string get_human_readable_time_diff(uint64_t other_time);
};