#include "catch.hpp"
#include "TimerFD.hpp"
#include <unistd.h>

TEST_CASE( "TimerFD_accuracy" ) {

    const uint64_t period = 21000000;
    const uint64_t offset =  5000000;
    TimerFD timer(period, offset);

    int count = 0;
    uint64_t t_start_prev = 0;

    timer.start([&](uint64_t t_start){
        uint64_t now = timer.get_time();

        CHECK( t_start <= now );
        CHECK( now <= t_start + 1000000 ); // actual start time is within 1 ms of declared start time
        CHECK( t_start % period == offset ); // start time corresponds to timer definition

        if(count > 0)
        {
            CHECK( ((count%3)+1)*period == t_start - t_start_prev);
        }

        count++;
        if(count > 15) {
            timer.stop();
        }

        t_start_prev = t_start;

        usleep( ((count%3)*period + period/3) / 1000 ); // simluate variable runtime
    });
}
