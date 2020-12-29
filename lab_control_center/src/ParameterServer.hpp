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

#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <atomic>

#include "Parameter.hpp"
#include "ParameterRequest.hpp"
#include "ParameterStorage.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Writer.hpp"
#include "ParameterWithDescription.hpp"

#include "cpm/AsyncReader.hpp"

/**
 * \class ParameterServer
 * Server that distributes parameter values. This server uses ParameterStorage to store its values.
 * \ingroup lcc
*/
class ParameterServer {
private:    
    //Callback
    void handleParamRequest(std::vector<ParameterRequest>& samples);
    void handleSingleParamRequest(std::string name);

    //Communication
    cpm::Writer<Parameter> writer;
    cpm::AsyncReader<ParameterRequest> readerParameterRequest;

    std::thread delayed_init_param_thread;

public:
    ParameterServer(std::shared_ptr<ParameterStorage> _storage);
    ~ParameterServer();

    void resend_all_params();

    void resend_param_callback(std::string name);
    
    std::shared_ptr<ParameterStorage> storage;
};