#pragma once

#include "cpm/Timer.hpp"
#include <thread>

class TimerFD : public cpm::Timer
{
    bool active = false;
    int timer_fd = -1;
    uint64_t period_nanoseconds; 
    uint64_t offset_nanoseconds;
    std::thread runner_thread;
    std::function<void(uint64_t t_now)> m_update_callback;

    void wait();

public:
    TimerFD(uint64_t period_nanoseconds, uint64_t offset_nanoseconds);
    ~TimerFD();

    void start       (std::function<void(uint64_t t_now)> update_callback) override;
    void start_async (std::function<void(uint64_t t_now)> update_callback) override;
    void stop() override;
    uint64_t get_time() override;
};