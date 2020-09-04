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

#include "RTTAggregator.hpp"

RTTAggregator::RTTAggregator()
{
    create_rtt_thread();
}

RTTAggregator::~RTTAggregator()
{
    destroy_rtt_thread();
}

void RTTAggregator::create_rtt_thread()
{
    //Create thread to measure RTT regularly
    run_rtt_thread.store(true);
    check_rtt_thread = std::thread(
        [&](){
            while(run_rtt_thread.load())
            {
                //No waiting required, the RTTTool function itself already includes over 0.5s of waiting times
                auto rtt = cpm::RTTTool::Instance().measure_rtt();

                //Store new values, if they exist
                if (rtt.first > 0)
                {
                    std::lock_guard<std::mutex> lock(rtt_values_mutex);
                    rtt_was_measured = true;

                    current_best_rtt = rtt.first;
                    current_worst_rtt = rtt.second;
                    all_time_worst_rtt = std::max(current_worst_rtt, all_time_worst_rtt);
                }
            }
        }
    );
}

void RTTAggregator::destroy_rtt_thread()
{
    run_rtt_thread.store(false);
    if (check_rtt_thread.joinable())
    {
        check_rtt_thread.join();
    }
}

void RTTAggregator::restart_measurement()
{
    destroy_rtt_thread();
    create_rtt_thread();
}

void RTTAggregator::stop_measurement()
{
    destroy_rtt_thread();
}

bool RTTAggregator::get_rtt(uint64_t &_current_best_rtt, uint64_t &_current_worst_rtt, uint64_t &_all_time_worst_rtt)
{
    std::lock_guard<std::mutex> lock(rtt_values_mutex);

    _current_best_rtt = current_best_rtt;
    _current_worst_rtt = current_worst_rtt;
    _all_time_worst_rtt = all_time_worst_rtt;

    return rtt_was_measured;
}