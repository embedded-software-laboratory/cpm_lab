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

/**
 * \file HLCReadyAggregator.cpp
 * \ingroup lcc
 */

//The reader callback is initialized in the init list of the constructor; store all IDs in a map together with the current time in nanoseconds
HLCReadyAggregator::HLCReadyAggregator() :
    async_hlc_reader(
        [&](std::vector<HLCHello>& samples){
            //Lock the mutex for thread-safe access
            std::lock_guard<std::mutex> lock(hlc_list_mutex);

            //Store new IDs / update receive time
            //Checks for on-/offline NUCs are performed in get_hlc_ids_string, which is called regularly by the UI
            for (auto& data : samples)
            {
                auto id_string = data.source_id();
                //Convert ID to uint8_t
                try {
                    int id_int = std::stoi(id_string);
                    if (id_int < 0 || id_int > 255)
                    {
                        throw std::runtime_error("HLC ID is too small / too large");
                    }

                    uint8_t id_uint8 = static_cast<uint8_t>(id_int);
                    
                    hlc_map[id_uint8] = cpm::get_time_ns();

                    //Store whether the programs on the HLC are currently running (with a small risk that the order of msgs is not correct)
                    hlc_script_running[id_uint8] = data.script_running();
                    hlc_middleware_running[id_uint8] = data.middleware_running();
                }
                catch (const std::runtime_error& err)
                {
                    cpm::Logging::Instance().write(2, "Error on converting HLC ID %s: %s", id_string, err.what());
                }
                catch (...) {
                    cpm::Logging::Instance().write(2, "Error: Could not convert HLC ID %s to int in HLCReadyAggregator", id_string);
                }
            }
        },
        "hlc_hello",
        true)
{
}

std::vector<std::string> HLCReadyAggregator::get_hlc_ids_string()
{
    auto ids_uint = get_hlc_ids_uint8_t();

    std::vector<std::string> valid_hlc_ids;
    for (auto entry : ids_uint)
    {
        //Convert ID to string (convert to int first, or uint8_t is interpreted as symbol)
        valid_hlc_ids.push_back(std::to_string(static_cast<int>(entry)));
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

    for (auto iterator = hlc_map.begin(); iterator != hlc_map.end();)
    {
        if (current_time_ns - iterator->second < time_to_live_ns)
        {
            valid_hlc_ids.push_back(iterator->first);
            ++iterator;
        }
        else
        {
            cpm::Logging::Instance().write(1, "HLC / NUC crashed / now offline / missed online message: %s", std::to_string(static_cast<int>(iterator->first)).c_str());
            iterator = hlc_map.erase(iterator);
        }
        
    }

    return valid_hlc_ids;
}

bool HLCReadyAggregator::script_running_on(uint8_t hlc_id)
{
    std::lock_guard<std::mutex> lock(hlc_list_mutex);

    //Considered not running if no HLC msg has been received
    auto iterator = hlc_map.find(hlc_id);
    if (iterator == hlc_map.end())
    {
        return false;
    }

    //Considered not running if data is not up to date
    if (cpm::get_time_ns() - iterator->second >= time_to_live_ns)
    {
        return false;
    }

    //Else, obtain the actual value - which must exist if an entry in hlc_map exists 
    return hlc_script_running.at(hlc_id);
}

bool HLCReadyAggregator::middleware_running_on(uint8_t hlc_id)
{
    std::lock_guard<std::mutex> lock(hlc_list_mutex);

    //Considered not running if no HLC msg has been received
    auto iterator = hlc_map.find(hlc_id);
    if (iterator == hlc_map.end())
        return false;

    //Considered not running if data is not up to date
    if (cpm::get_time_ns() - iterator->second >= time_to_live_ns)
    {
        return false;
    }

    //Else, obtain the actual value - which must exist if an entry in hlc_map exists 
    return hlc_middleware_running.at(hlc_id);
}
