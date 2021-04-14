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

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <chrono>
#include <thread>

#include <unistd.h> //For usleep; Change to sleep_for is possible as soon as the ARM Build supports C++11

#include "dds/Parameter.hpp"
#include "dds/ParameterRequest.hpp"
#include "cpm/Logging.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/Writer.hpp"

namespace cpm
{
    /**
    * \brief Singleton that receives and stores constants, e.g. for configuration
    * Is used by to get data for the user (requests data from the param server or uses stored data if available)
    * \ingroup cpmlib
    */
    class ParameterReceiver {
    public:
        //! Provides access to the parameter receiver Singleton
        static ParameterReceiver& Instance();

        //Delete move and copy op
        ParameterReceiver(ParameterReceiver const&) = delete;
        ParameterReceiver(ParameterReceiver&&) = delete; 
        ParameterReceiver& operator=(ParameterReceiver const&) = delete;
        ParameterReceiver& operator=(ParameterReceiver &&) = delete;

        /**
         * \brief Access function to get a boolean parameter via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        bool parameter_bool(std::string parameter_name);

        /**
         * \brief Access function to get a uin64_t parameter via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        uint64_t parameter_uint64_t(std::string parameter_name);

        /**
         * \brief Access function to get an int32_t parameter via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        int32_t parameter_int(std::string parameter_name);

        /**
         * \brief Access function to get a double parameter via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        double parameter_double(std::string parameter_name);

        /**
         * \brief Access function to get a string parameter via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        std::string parameter_string(std::string parameter_name);

        /**
         * \brief Access function to get a list of int32_t parameter values via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        std::vector<int32_t> parameter_ints(std::string parameter_name);

        /**
         * \brief Access function to get a list of double parameter values via cpm::parameter_... functions
         * \param parameter_name the name of the parameter
         * \return the value of the parameter with the given name
         */
        std::vector<double> parameter_doubles(std::string parameter_name);

    private:
        /**
         * \brief Constructor. Creates a reliable DataWriter and uses the "is_reliable" parameter of the AsyncReader to create a reliable DataReader as well. Also binds the callback function / passes it to the AsyncReader.
         */
        ParameterReceiver();

        //Variable storage, DDS request is sent only if the storage for key 'parameter_name' is empty
        //! Param storage for boolean variables
        std::map<std::string, bool> param_bool;
        //! Param storage for uint64_t variables
        std::map<std::string, uint64_t> param_uint64_t;
        //! Param storage for int variables
        std::map<std::string, int32_t> param_int;
        //! Param storage for double variables
        std::map<std::string, double> param_double;
        //! Param storage for string variables
        std::map<std::string, std::string> param_string;
        //! Param storage for list-of-int variables
        std::map<std::string, std::vector<int32_t>> param_ints;
        //! Param storage for list-of-double variables
        std::map<std::string, std::vector<double>> param_doubles;

        //Mutex for each map
        //! Mutex for boolean param storage
        std::mutex param_bool_mutex;
        //! Mutex for uint64_t param storage
        std::mutex param_uint64_t_mutex;
        //! Mutex for int param storage
        std::mutex param_int_mutex;
        //! Mutex for double param storage
        std::mutex param_double_mutex;
        //! Mutex for string param storage
        std::mutex param_string_mutex;
        //! Mutex for list-of-int param storage
        std::mutex param_ints_mutex;
        //! Mutex for list-of-double param storage
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
        
        //! Internal writer to requrest parameter values
        cpm::Writer<ParameterRequest> writer;
        //! Internal async reader to receive any parameter values that are sent in the network
        cpm::AsyncReader<Parameter> subscriber;
    };

}