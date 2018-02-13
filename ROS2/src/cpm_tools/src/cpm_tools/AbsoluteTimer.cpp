#include "cpm_tools/AbsoluteTimer.hpp"

AbsoluteTimer::AbsoluteTimer(time_t seconds, long nanoseconds, std::function<void()> callback)
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

    struct itimerspec its;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1;
    its.it_interval.tv_sec = (time_t)seconds;
    its.it_interval.tv_nsec = nanoseconds;
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
    active = false;
    this->timer_thread.join();
}