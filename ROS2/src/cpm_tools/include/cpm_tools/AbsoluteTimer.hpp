#pragma once


#include <time.h>   
#include <functional>
#include <thread>
#include <sys/timerfd.h>
#include <unistd.h>

/*

A timer that calls the "callback" function repeatedly with a period 
specified by the "seconds" and "nanoseconds" arguments. 
The timer is absolute, i.e., it activates when the system clock is an
integer multiple of the period.
The callback must be thread-safe.

*/
class AbsoluteTimer {
    int fd;
    std::function<void()> callback_;
    bool active = true;
    std::thread timer_thread;

public:
    AbsoluteTimer(time_t seconds, long nanoseconds, std::function<void()> callback);
    void stop();
    ~AbsoluteTimer();
};
