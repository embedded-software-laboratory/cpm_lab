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

#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/Logging.hpp"
#include "cpm/TimerFD.hpp"
#include "TimerSimulated.hpp"

/**
 * \file Timer.cpp
 * \ingroup cpmlib
 */

namespace cpm {

std::shared_ptr<Timer> Timer::create(
    std::string node_id,
    uint64_t period_nanoseconds, 
    uint64_t offset_nanoseconds,
    bool wait_for_start, 
    bool simulated_time_allowed,
    bool simulated_time
) 
{
    // Switch between FD and simulated time
    if (simulated_time && simulated_time_allowed) {
        //Use timer for simulated time
        return std::make_shared<TimerSimulated>(node_id, period_nanoseconds, offset_nanoseconds);
    }
    else if (simulated_time && !simulated_time_allowed) {
        Logging::Instance().write(
            1,
            "%s", 
            "Timer Error: simulated time requested but not allowed."
        );
        fprintf(stderr, "Error: simulated time requested but not allowed.\n");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
    else {
        //Use timer for real time
        return std::make_shared<TimerFD>(node_id, period_nanoseconds, offset_nanoseconds, wait_for_start);
    }
}


}