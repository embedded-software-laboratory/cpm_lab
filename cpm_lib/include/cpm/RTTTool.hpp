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

#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"

#include "RoundTripTime.hpp"

#include "dds/pub/DataWriter.hpp"

namespace cpm
{
    /**
     * \brief This class (singleton) is used to create a thread that automatically runs in the background, 
     * which replies to every round trip time message received immediately with the current program's logging ID
     * 
     * It can also be used to measure the round trip time
     */
    class RTTTool
    {
        RTTTool(RTTTool const&) = delete;
        RTTTool(RTTTool&&) = delete; 
        RTTTool& operator=(RTTTool const&) = delete;
        RTTTool& operator=(RTTTool &&) = delete;

    private:
        dds::topic::Topic<RoundTripTime> rtt_topic;
        dds::pub::DataWriter<RoundTripTime> rtt_writer;

        std::shared_ptr<cpm::AsyncReader<RoundTripTime>> rtt_reader;

        std::string program_id = "no_prog_id_set";

        //Measure RTT request receive times, read async, requested by measure_rtt
        std::atomic_bool rtt_measure_requested;
        std::atomic_uint8_t rtt_count;
        std::mutex receive_times_mutex;
        std::vector<uint64_t> receive_times;

        /**
         * \brief Constructor - private bc singleton
         */
        RTTTool();

    public:
        /**
         * \brief Interface to create / get access to the singleton
         */ 
        static RTTTool& Instance();

        /**
         * \brief Set the ID of RTT msgs here (should already be done in internal configuration, same as log ID, but can be changed afterwards if desired)
         * \param _program_id ID set for the answers to round trip time messages, to identify the program (type) (e.g. vehicle (5), middleware, ...)
         */
        void set_id(std::string _program_id);

        /**
         * \brief Use this function to measure the (best and worst) round trip time in your network in ns
         * WARNING: Make sure that this function is only used by one program in your whole network, 
         * or you might get wrong results due to bad timing of both functions!
         * \return {0, 0} in case of an error / no answer, else the best and 'worst' (within 500ms) measured RTT
         */
        std::pair<uint64_t, uint64_t> measure_rtt();
    };
};