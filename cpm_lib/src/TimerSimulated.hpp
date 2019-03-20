#pragma once

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include "cpm/exceptions.hpp"

#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

class TimerSimulated : public cpm::Timer
{
    uint64_t period_nanoseconds; 
    uint64_t offset_nanoseconds;
    dds::topic::Topic<ReadyStatus> ready_topic;
    dds::topic::Topic<SystemTrigger> trigger_topic;
    dds::pub::DataWriter<ReadyStatus> writer_ready_status;
    dds::sub::DataReader<SystemTrigger> reader_system_trigger;
    std::string node_id;
    uint64_t current_time;

    bool active = false;

    std::thread runner_thread;
    std::function<void(uint64_t t_now)> m_update_callback;

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
    void start_async (std::function<void(uint64_t t_now)> update_callback) override;
    void stop() override;
    uint64_t get_time() override;
};