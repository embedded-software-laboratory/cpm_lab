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

#include "ParameterServer.hpp"
#include "cpm/get_topic.hpp"

using namespace std::placeholders;

ParameterServer::ParameterServer(std::shared_ptr<ParameterStorage> _storage):
    writer("parameter", true),
    readerParameterRequest(
        std::bind(&ParameterServer::handleParamRequest, this, _1), 
        "parameterRequest"
    ),
    storage(_storage)
{
    delayed_init_param_thread = std::thread([&]() {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        resend_all_params();
    });
}

ParameterServer::~ParameterServer() {
    if(delayed_init_param_thread.joinable()) {
        delayed_init_param_thread.join();
    }
}

void ParameterServer::resend_all_params() {
    std::vector<ParameterWithDescription> all_params = storage->get_all_parameters();

    for (auto param : all_params) {
        handleSingleParamRequest(param.parameter_data.name());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void ParameterServer::resend_param_callback(std::string name) {
    handleSingleParamRequest(name);
}

void ParameterServer::handleParamRequest(std::vector<ParameterRequest>& samples) {
    for (auto& data : samples) {
        handleSingleParamRequest(data.name());
    }
}

void ParameterServer::handleSingleParamRequest(std::string name) {
    Parameter param = Parameter();
    param.name(name);

    bool boolParam;
    if(storage->get_parameter_bool(name, boolParam)) {
        param.type(ParameterType::Bool);
        param.value_bool(boolParam);

        writer.write(param);
        return;
    }

    int32_t intParam;
    if(storage->get_parameter_int(name, intParam)) {
        //Create data to send
        std::vector<int32_t> stdInts;
        stdInts.push_back(intParam);
        rti::core::vector<int32_t> ints(stdInts);

        param.type(ParameterType::Int32);
        param.values_int32(ints);
        
        //Send new value
        writer.write(param);
        return;
    }

    double doubleParam;
    if(storage->get_parameter_double(name, doubleParam)) {
        //Create data to send
        std::vector<double> stdDoubles;
        stdDoubles.push_back(doubleParam);
        rti::core::vector<double> doubles(stdDoubles);

        param.type(ParameterType::Double);
        param.values_double(doubles);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::string stringParam;
    if(storage->get_parameter_string(name, stringParam)) {
        param.type(ParameterType::String);
        param.value_string(stringParam);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::vector<int32_t> intParams;
    if(storage->get_parameter_ints(name, intParams)) {
        //Create data to send
        rti::core::vector<int32_t> ints(intParams);

        param.type(ParameterType::Vector_Int32);
        param.values_int32(ints);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::vector<double> doubleParams;
    if(storage->get_parameter_doubles(name, doubleParams)) {
        //Create data to send
        rti::core::vector<double> doubles(doubleParams);

        param.type(ParameterType::Vector_Double);
        param.values_double(doubles);
        
        //Send new value
        writer.write(param);
        return;
    }
}