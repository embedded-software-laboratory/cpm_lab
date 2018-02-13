#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cpm_tools/AbsoluteTimer.hpp>

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