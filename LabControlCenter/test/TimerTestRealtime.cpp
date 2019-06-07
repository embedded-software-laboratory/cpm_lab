#include <chrono>
#include <iostream>
#include <thread>
#include "cpm/Timer.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"

//In this test scenario, the timers are not stopped by the program but by the LCC stop signal

int main(int argc, char *argv[]) {
    cpm::Logging::Instance().set_id("Logger_test");

    std::cout << "Creating timers..." << std::endl;

    std::string id = cpm::cmd_parameter_string("id", "sim_test", argc, argv);
    int period = cpm::cmd_parameter_int("period", 1000000, argc, argv);

    auto timer1 = cpm::Timer::create(id, static_cast<uint64_t>(period), 1, true, false, false);
    timer1->start([&](uint64_t t_now) {
        std::cout << "Time now: " << t_now << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    std::cout << "Shutting down..." << std::endl;

    return 0;
}