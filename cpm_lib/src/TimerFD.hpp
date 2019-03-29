#pragma once

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

#include "cpm/exceptions.hpp"

class TimerFD : public cpm::Timer
{
    uint64_t period_nanoseconds; 
    uint64_t offset_nanoseconds;
    dds::topic::Topic<ReadyStatus> ready_topic;
    dds::topic::Topic<SystemTrigger> trigger_topic;
    std::string node_id;
    dds::sub::DataReader<SystemTrigger> reader_system_trigger;

    dds::sub::cond::ReadCondition readCondition;
    dds::core::cond::WaitSet waitset;
    
    bool active = false;
    int timer_fd = -1;
    std::thread runner_thread;
    std::function<void(uint64_t t_now)> m_update_callback;


    void wait();
    uint64_t receiveStartTime(); //Bool: true if start signal was received, false if stop signal was received
    bool received_stop_signal ();
    
    const bool wait_for_start; //If false, do not use receiveStartTime()

public:
    TimerFD(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool wait_for_start);
    ~TimerFD();

    void createTimer ();
    void start       (std::function<void(uint64_t t_now)> update_callback) override;
    void start_async (std::function<void(uint64_t t_now)> update_callback) override;
    void stop() override;
    uint64_t get_time() override;
};