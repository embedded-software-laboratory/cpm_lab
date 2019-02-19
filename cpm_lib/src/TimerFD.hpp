#pragma once

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

#include "ErrorTimerStart.hpp"

class TimerFD : public cpm::Timer
{
    std::string node_id;
    bool active = false;
    int timer_fd = -1;
    uint64_t period_nanoseconds; 
    uint64_t offset_nanoseconds;
    std::thread runner_thread;
    std::function<void(uint64_t t_now)> m_update_callback;
    //Topics need to be created before the test case is used (as it must be able to access the topic)
    dds::topic::Topic<ReadyStatus> ready_topic;
    dds::topic::Topic<SystemTrigger> trigger_topic;
    dds::sub::DataReader<SystemTrigger> reader;

    void wait();
    void waitForStart();

    uint64_t two;
    uint64_t max_time;
    bool wait_for_start; //If false, do not use waitForStart()

public:
    TimerFD(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool wait_for_start);
    ~TimerFD();

    void createTimer ();
    void start       (std::function<void(uint64_t t_now)> update_callback) override;
    void start_async (std::function<void(uint64_t t_now)> update_callback) override;
    void stop() override;
    bool got_stop_signal ();
    uint64_t get_time() override;
};