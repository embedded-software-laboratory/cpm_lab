#pragma once

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

class TimerSimulated : public cpm::Timer
{
    std::string node_id;
    bool active = false;
    int timer_fd = -1;
    uint64_t period_nanoseconds; 
    uint64_t offset_nanoseconds;
    std::thread runner_thread;
    std::function<void(uint64_t t_now)> m_update_callback;

    void wait();
    void waitForStart();

public:
    TimerSimulated(std::string _node_id, uint64_t period_nanoseconds, uint64_t offset_nanoseconds);
    ~TimerSimulated();

    void start       (std::function<void(uint64_t t_now)> update_callback) override;
    void start_async (std::function<void(uint64_t t_now)> update_callback) override;
    void stop() override;
    uint64_t get_time() override;
};