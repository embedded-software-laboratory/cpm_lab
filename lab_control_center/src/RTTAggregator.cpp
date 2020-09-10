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

                std::lock_guard<std::mutex> lock(rtt_values_mutex);
                std::set<std::string> missing_ids = received_ids;

                //Store new values, if they exist
                if (rtt.size() > 0)
                {
                    //Update data (if it exists)
                    for (auto& entry : rtt)
                    {
                        //Remember new IDs (std::set is used, so old IDs are 'ignored' here)
                        received_ids.insert(entry.first);
                        
                        //Also, remember for which IDs data is missing
                        missing_ids.erase(entry.first);

                        //Store new data
                        current_best_rtt[entry.first] = entry.second.first;
                        current_worst_rtt[entry.first] = entry.second.second;

                        //Special behaviour in case this is the first data
                        if (measure_count.find(entry.first) == measure_count.end())
                        {
                            missed_answers[entry.first] = 0;
                            measure_count[entry.first] = 1;

                            all_time_worst_rtt[entry.first] = entry.second.second;
                        }
                        else
                        {
                            measure_count[entry.first] += 1;

                            all_time_worst_rtt[entry.first] = std::max(entry.second.second, all_time_worst_rtt[entry.first]);
                        }
                    }
                }

                //Store data for missing IDs
                for (auto& id : missing_ids)
                {
                    //Store new data
                    current_best_rtt[id] = 0;
                    current_worst_rtt[id] = 0;

                    if (measure_count.find(id) == measure_count.end())
                    {
                        missed_answers[id] = 1;
                        measure_count[id] = 1;
                    }
                    else
                    {
                        missed_answers[id] += 1;
                        measure_count[id] += 1;
                    }
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

    //Clear all previous data
    std::lock_guard<std::mutex> lock(rtt_values_mutex);
    current_best_rtt.clear();
    current_worst_rtt.clear();
    all_time_worst_rtt.clear();
    measure_count.clear();
    missed_answers.clear();
    received_ids.clear();

    create_rtt_thread();
}

void RTTAggregator::stop_measurement()
{
    destroy_rtt_thread();
}

bool RTTAggregator::get_rtt(std::string participant_id, uint64_t &_current_best_rtt, uint64_t &_current_worst_rtt, uint64_t &_all_time_worst_rtt, double &_missed_answers)
{
    std::lock_guard<std::mutex> lock(rtt_values_mutex);

    //Return if no values were yet measured for this participant
    if (current_best_rtt.find(participant_id) == current_best_rtt.end())
    {
        return false;
    }

    _current_best_rtt = current_best_rtt[participant_id];
    _current_worst_rtt = current_worst_rtt[participant_id];
    _all_time_worst_rtt = all_time_worst_rtt[participant_id];
    //Should never lead to errors, as the value in measure_count is always greater than 1
    //Thus, if that is the case, an overflow must have occured already before that
    if (measure_count[participant_id] <= 0)
    {
        _missed_answers = 0;
        cpm::Logging::Instance().write(1, "%s", "Warning: An overflow occured in RTT aggregator, missed answers data is certainly wrong");
    }
    else
    {
        _missed_answers = missed_answers[participant_id] / measure_count[participant_id];
    }

    return true;
}