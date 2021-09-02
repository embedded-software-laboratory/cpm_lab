#include <chrono>
#include <iostream>
#include <thread>
#include "cpm/Timer.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"


/**
 * \file TimerTestRealtime.cpp
 * \brief Test scenario: Creates a real-time timer that should be visible and stoppable from the LCC' timer tab
 * \ingroup lcc
 */

int main(int argc, char *argv[]) {
    cpm::Logging::Instance().set_id("Logger_test");

    std::cout << "Creating timers..." << std::endl;

    std::string id = cpm::cmd_parameter_string("id", "sim_test", argc, argv);
    uint64_t period = cpm::cmd_parameter_uint64_t("period", 1000000ull, argc, argv);

    auto timer1 = cpm::Timer::create(id, period, 1ull, true, false, false);
    timer1->start([&](uint64_t t_now) {
        std::cout << "Time now: " << t_now << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    std::cout << "Shutting down..." << std::endl;

    return 0;
}