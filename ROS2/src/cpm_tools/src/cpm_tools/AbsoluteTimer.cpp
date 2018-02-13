#include "cpm_tools/AbsoluteTimer.hpp"

AbsoluteTimer::AbsoluteTimer(
    time_t period_seconds, long period_nanoseconds, 
    time_t offset_seconds, long offset_nanoseconds, 
    std::function<void()> callback)
:callback_(callback)
{
    // Timer setup
    fd = timerfd_create(CLOCK_REALTIME, 0);
    if (fd == -1) {
        fprintf(stderr, "Call to timerfd_create failed.\n"); 
        perror("timerfd_create");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

    if(offset_seconds == 0 && offset_nanoseconds == 0) {
        offset_nanoseconds = 1;
    }

    struct itimerspec its;
    its.it_value.tv_sec = offset_seconds;
    its.it_value.tv_nsec = offset_nanoseconds;
    its.it_interval.tv_sec = period_seconds;
    its.it_interval.tv_nsec = period_nanoseconds;
    int status = timerfd_settime(fd, TFD_TIMER_ABSTIME, &its, NULL);
    if (status != 0) {
        fprintf(stderr, "Call to timer_settime returned error status (%d).\n", status);
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

    // Timer event loop
    this->timer_thread = std::thread([&](){
        while(this->active) {
            unsigned long long missed;
            int status = read(fd, &missed, sizeof(missed));
            if(status != sizeof(missed)) {
                fprintf(stderr, "Error: read(timerfd), status %d.\n", status);
                fflush(stderr); 
                exit(EXIT_FAILURE);
            }
            if(callback_) callback_();
        }
    });
}

void AbsoluteTimer::stop() {
    if(active) {
        active = false;
        this->timer_thread.join();
    }
}


AbsoluteTimer::~AbsoluteTimer() {
    if(active) {
        active = false;
        this->timer_thread.join();
    }  
}