#include "catch.hpp"
#include "TimerFD.hpp"
#include "cpm/exceptions.hpp"
#include <unistd.h>

#include <thread>
#include <string>
#include <vector>

#include "cpm/ParticipantSingleton.hpp"
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

/**
 * Tests:
 * - Calls start(_async) after the timer has been started -> exceptions should be thrown
 */

TEST_CASE( "TimerFD_start_again" ) {
    //Set the Logger ID
    Logging::Instance().set_id("test_timerfd_start_again");

    const uint64_t period = 21000000;
    const uint64_t offset =  0;

    std::string timer_id = "2";
    TimerFD timer(timer_id, period, offset, false);


    //Check if start_async works as expected as well and store timestamps
    timer.start_async([&](uint64_t t_start){
        usleep(100);
    });

    //Check that the timer cannot be used while it is running
    usleep(1000000);
    CHECK_THROWS_AS(timer.start([](uint64_t t_start) {}), cpm::ErrorTimerStart);
    CHECK_THROWS_AS(timer.start_async([](uint64_t t_start) {}), cpm::ErrorTimerStart);


    timer.stop();

}
