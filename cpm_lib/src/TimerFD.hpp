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
    uint64_t period_nanoseconds; 
    uint64_t offset_nanoseconds;
    dds::topic::Topic<ReadyStatus> ready_topic;
    dds::topic::Topic<SystemTrigger> trigger_topic;
    std::string node_id;
    dds::sub::DataReader<SystemTrigger> reader;
    
    bool active = false;
    int timer_fd = -1;
    std::thread runner_thread;
    std::function<void(uint64_t t_now)> m_update_callback;


    void wait();
    bool waitForStart(); //Bool: true if start signal was received, false if stop signal was received
    
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