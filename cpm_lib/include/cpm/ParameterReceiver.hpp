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

/*
 * Singleton that receives and stores constants, e.g. for configuration
 * Is used by to get data for the user (requests data from the param server or uses stored data if available)
 */

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <chrono>

#include "dds/Parameter.hpp"
#include "dds/ParameterRequest.hpp"
#include "cpm/Logging.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/Writer.hpp"

namespace cpm
{

    class ParameterReceiver {
    public:
        static ParameterReceiver& Instance();

        //Delete move and copy op
        ParameterReceiver(ParameterReceiver const&) = delete;
        ParameterReceiver(ParameterReceiver&&) = delete; 
        ParameterReceiver& operator=(ParameterReceiver const&) = delete;
        ParameterReceiver& operator=(ParameterReceiver &&) = delete;

        //Provide access similar to the interface
        bool parameter_bool(std::string parameter_name);
        int32_t parameter_int(std::string parameter_name);
        double parameter_double(std::string parameter_name);
        std::string parameter_string(std::string parameter_name);
        std::vector<int32_t> parameter_ints(std::string parameter_name);
        std::vector<double> parameter_doubles(std::string parameter_name);

    private:
        /**
         * \brief Constructor. Creates a reliable DataWriter and uses the "is_reliable" parameter of the AsyncReader to create a reliable DataReader as well. Also binds the callback function / passes it to the AsyncReader.
         */
        ParameterReceiver();

        //Variable storage, DDS request is sent only if the storage for key 'parameter_name' is empty
        std::map<std::string, bool> param_bool;
        std::map<std::string, int32_t> param_int;
        std::map<std::string, double> param_double;
        std::map<std::string, std::string> param_string;
        std::map<std::string, std::vector<int32_t>> param_ints;
        std::map<std::string, std::vector<double>> param_doubles;

        //Mutex for each map
        std::mutex param_bool_mutex;
        std::mutex param_int_mutex;
        std::mutex param_double_mutex;
        std::mutex param_string_mutex;
        std::mutex param_ints_mutex;
        std::mutex param_doubles_mutex;

        /**
         * \brief Sends a param request to the param server with the given parameter name
         * \param parameter_name Name of the requested param
         */
        void requestParam(std::string parameter_name);

        /**
         * \brief Callback function that handles incoming parameter definitions. Parameters are stored in maps depending on their type and name for later access.
         * \param samples Samples to be processed by the callback function (received messages)
         */
        void callback(std::vector<Parameter>& samples);

    private:
        cpm::Writer<ParameterRequest> writer;
        cpm::AsyncReader<Parameter> subscriber;
    };

}