#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <time.h>   
#include <functional>
#include <thread>



class AbsoluteTimer {
    int fd;
    std::function<void()> callback_;
    bool active = true;
    std::thread timer_thread;

public:
    AbsoluteTimer(int32_t seconds, uint32_t nanoseconds, std::function<void()> callback)
    :callback_(callback)
    {
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

    void stop() {
        active = false;
        this->timer_thread.join();
    }
};




int main(int argc, char *argv[])
{
    AbsoluteTimer at(1, 0, [&](){
        struct timespec start;
        clock_gettime(CLOCK_REALTIME, &start);
        printf("%li.%09li\n", start.tv_sec, start.tv_nsec);
        fflush(stdout); 
    });
    sleep(60);
    at.stop();
    return 0;
}