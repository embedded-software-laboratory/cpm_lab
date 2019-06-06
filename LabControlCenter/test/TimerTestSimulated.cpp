#include <iostream>
#include "cpm/Timer.hpp"

//In this test scenario, the timers are not stopped by the program but by the LCC stop signal

int main(int argc, char *argv[]) {
    std::cout << "Creating timers..." << std::endl;

    auto timer1 = cpm::Timer::create("sim_test_1", 1000000, 2, true, true, true);
    timer1->start_async([&](uint64_t t_now) {

    });

    auto timer2 = cpm::Timer::create("sim_test_1", 1000000, 2, true, true, true);
    timer2->start_async([&](uint64_t t_now) {

    });

    std::cout << "Shutting down..." << std::endl;

    return 0;
}