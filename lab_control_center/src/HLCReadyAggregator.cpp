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

#include "HLCReadyAggregator.hpp"

//The reader callback is initialized in the init list of the constructor; store all IDs in a map together with the current time in nanoseconds
HLCReadyAggregator::HLCReadyAggregator() :
    async_hlc_reader(
        [&](dds::sub::LoanedSamples<ReadyStatus>& samples){
            //Lock the mutex for thread-safe access
            std::lock_guard<std::mutex> lock(hlc_list_mutex);

            //Store new IDs / update receive time
            for (auto sample : samples)
            {
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    hlc_map[data.source_id()] = cpm::get_time_ns();
                }
            }
        },
        cpm::ParticipantSingleton::Instance(),
        cpm::get_topic<ReadyStatus>("hlc_startup"))
{
}

std::vector<std::string> HLCReadyAggregator::get_hlc_ids_string()
{
    //Lock the mutex for thread-safe access
    std::lock_guard<std::mutex> lock(hlc_list_mutex);

    //Only use IDs that are still up-to-date
    uint64_t current_time_ns = cpm::get_time_ns();
    std::vector<std::string> valid_hlc_ids;
    for (auto entry : hlc_map)
    {
        if (current_time_ns - entry.second < time_to_live_ns)
        {
            valid_hlc_ids.push_back(entry.first);
        }
    }

    return valid_hlc_ids;
}

std::vector<uint8_t> HLCReadyAggregator::get_hlc_ids_uint8_t()
{
    //Lock the mutex for thread-safe access
    std::lock_guard<std::mutex> lock(hlc_list_mutex);

    //Only use IDs that are still up-to-date
    uint64_t current_time_ns = cpm::get_time_ns();
    std::vector<uint8_t> valid_hlc_ids;
    for (auto entry : hlc_map)
    {
        if (current_time_ns - entry.second < time_to_live_ns)
        {
            //Convert ID to uint8_t
            try {
                int int_value = std::stoi(entry.first);
                valid_hlc_ids.push_back(static_cast<uint8_t>(int_value));
            }
            catch (...) {
                std::cerr << "Error: Could not convert HLC ID '" << entry.first << "' to int" << std::endl;
            }
        }
    }

    return valid_hlc_ids;
}