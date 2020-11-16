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
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Writer.hpp"

#include "RoundTripTime.hpp"

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
        cpm::Writer<RoundTripTime> rtt_writer;

        std::shared_ptr<cpm::AsyncReader<RoundTripTime>> rtt_reader;

        std::string program_id = "no_prog_id_set";

        //Measure RTT request receive times, read async, requested by measure_rtt
        std::atomic_bool rtt_measurement_active;
        std::atomic_bool rtt_measure_requested;
        std::atomic<std::uint8_t> rtt_count; //Cannot use atomic_uint8_t due to compatability to lower C++ standards for vehicles
        std::mutex receive_times_mutex;
        std::map<std::string, std::vector<uint64_t>> receive_times;

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
         * \brief Activate the RTT measurement for this participant
         * It will answer to RTT requests with the ID set in the parameter
         * If not activated, RTT requests are being ignored by the participant
         * Must also be called for the RTT measurer, to set a correct program id for RTT requests
         * \param _program_id ID set for the answers to round trip time messages, to identify the program (type) (e.g. vehicle (5), middleware, ...)
         */
        void activate(std::string _program_id);

        /**
         * \brief Use this function to measure the (best and worst) round trip time in your network in ns
         * WARNING: Make sure that this function is only used by one program in your whole network, 
         * or you might get wrong results due to bad timing of both functions!
         * \return empty map / missing entries in case of an error / missing answers, else the best and 'worst' (within 2200ms) measured RTT for each participant id
         */
        std::map<std::string, std::pair<uint64_t, uint64_t>> measure_rtt();
    };
};