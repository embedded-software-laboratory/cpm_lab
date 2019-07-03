#pragma once

/*
 * ParameterServer.hpp
 * Server that distributes parameter values. This server uses ParameterStorage to store its values.
*/

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
#include "ParameterWithDescription.hpp"

#include "cpm/AsyncReader.hpp"
#include <dds/pub/ddspub.hpp>

class ParameterServer {
private:    
    //Callback
    void handleParamRequest(dds::sub::LoanedSamples<ParameterRequest>& samples);
    void handleSingleParamRequest(std::string name);

    //Communication
    dds::topic::Topic<Parameter> parameterTopic;
    dds::topic::Topic<ParameterRequest> parameterRequestTopic;
    dds::pub::DataWriter<Parameter> writer;
    cpm::AsyncReader<ParameterRequest> readerParameterRequest;

    std::thread delayed_init_param_thread;

public:
    ParameterServer(std::shared_ptr<ParameterStorage> _storage);
    ~ParameterServer();

    void resend_all_params();

    void resend_param_callback(std::string name);
    
    std::shared_ptr<ParameterStorage> storage;
};