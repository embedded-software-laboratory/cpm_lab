// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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