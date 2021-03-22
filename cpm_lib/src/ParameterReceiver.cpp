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

#include "cpm/ParameterReceiver.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/get_topic.hpp"
#include <chrono>
#include <thread>

using namespace std::placeholders;

/**
 * \file ParameterReceiver.cpp
 * \ingroup cpmlib
 */
namespace cpm
{

    bool parameter_bool(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_bool(parameter_name);
    }

    uint64_t parameter_uint64_t(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_uint64_t(parameter_name);
    }

    int32_t parameter_int(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_int(parameter_name);
    }

    double parameter_double(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_double(parameter_name);
    }

    std::string parameter_string(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_string(parameter_name);
    }

    std::vector<int32_t> parameter_ints(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_ints(parameter_name);
    }

    std::vector<double> parameter_doubles(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_doubles(parameter_name);
    }

    ParameterReceiver::ParameterReceiver():
        writer("parameterRequest", true),
        subscriber(std::bind(&ParameterReceiver::callback, this, _1), "parameter", true)
    {

    }

    ParameterReceiver& ParameterReceiver::Instance() {
        // Thread-safe in C++11
        static ParameterReceiver myInstance;
        return myInstance;
    }

    bool ParameterReceiver::parameter_bool(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_bool_mutex); 

        while (param_bool.find(parameter_name) == param_bool.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            usleep(1000);
            s_lock.lock();
        }

        bool retValue = param_bool.at(parameter_name);
        s_lock.unlock();
        return retValue;
    }

    uint64_t ParameterReceiver::parameter_uint64_t(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_uint64_t_mutex); 

        while (param_uint64_t.find(parameter_name) == param_uint64_t.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
            s_lock.lock();
        }

        uint64_t retValue = param_uint64_t.at(parameter_name);
        s_lock.unlock();
        return retValue;
    }

    int32_t ParameterReceiver::parameter_int(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_int_mutex); 

        while (param_int.find(parameter_name) == param_int.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            usleep(1000);
            s_lock.lock();
        }

        int32_t retValue = param_int.at(parameter_name);
        s_lock.unlock();
        return retValue;
    }

    double ParameterReceiver::parameter_double(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_double_mutex); 

        while (param_double.find(parameter_name) == param_double.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            usleep(1000);
            s_lock.lock();
        }

        double retValue = param_double.at(parameter_name);
        s_lock.unlock();
        return retValue;
    }

    std::string ParameterReceiver::parameter_string(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_string_mutex); 

        while (param_string.find(parameter_name) == param_string.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            usleep(1000);
            s_lock.lock();
        }

        std::string retValue = param_string.at(parameter_name);
        s_lock.unlock();
        return retValue;
    }

    std::vector<int32_t> ParameterReceiver::parameter_ints(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_ints_mutex); 

        while (param_ints.find(parameter_name) == param_ints.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            usleep(1000);
            s_lock.lock();
        }

        std::vector<int32_t> retValue(param_ints.at(parameter_name));
        s_lock.unlock();
        return retValue;
    }

    std::vector<double> ParameterReceiver::parameter_doubles(std::string parameter_name) {
        std::unique_lock<std::mutex> s_lock(param_doubles_mutex); 

        while (param_doubles.find(parameter_name) == param_doubles.end()) {
            s_lock.unlock();
            requestParam(parameter_name);
            Logging::Instance().write(
                2,
                "Waiting for parameter %s ...", 
                parameter_name.c_str()
            );
            usleep(1000);
            s_lock.lock();
        }

        std::vector<double> retValue(param_doubles.at(parameter_name));
        s_lock.unlock();
        return retValue;
    }

    void ParameterReceiver::requestParam(std::string parameter_name) {
        ParameterRequest request;
        request.name(parameter_name);
        writer.write(request);
    }

    void ParameterReceiver::callback(std::vector<Parameter>& samples) {
        for (const auto& parameter : samples) {
            if (parameter.type() == ParameterType::Int32 && parameter.values_int32().size() == 1) {
                std::lock_guard<std::mutex> u_lock(param_int_mutex);
                param_int[parameter.name()] = parameter.values_int32().at(0);
            }
            else if (parameter.type() == ParameterType::Double && parameter.values_double().size() == 1) {
                std::lock_guard<std::mutex> u_lock(param_double_mutex);
                param_double[parameter.name()] = parameter.values_double().at(0);
            }
            else if (parameter.type() == ParameterType::String) {
                std::lock_guard<std::mutex> u_lock(param_string_mutex);
                param_string[parameter.name()] = parameter.value_string();
            }
            else if (parameter.type() == ParameterType::Bool) {
                std::lock_guard<std::mutex> u_lock(param_bool_mutex);
                param_bool[parameter.name()] = parameter.value_bool();
            }
            else if (parameter.type() == ParameterType::UInt64) {
                    std::lock_guard<std::mutex> u_lock(param_uint64_t_mutex);
                    param_uint64_t[parameter.name()] = parameter.value_uint64_t();
                }
            else if (parameter.type() == ParameterType::Vector_Int32) {
                std::lock_guard<std::mutex> u_lock(param_ints_mutex);
                param_ints[parameter.name()] = parameter.values_int32();
            }
            else if (parameter.type() == ParameterType::Vector_Double) {
                std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
                param_doubles[parameter.name()] = parameter.values_double();
            }
        }
    }
}