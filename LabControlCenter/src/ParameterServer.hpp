#pragma once

/*
 * ParameterServer.hpp
 * Server that distributes parameter values. This server uses ParameterStorage to store its values.
*/

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <atomic>

#include "Parameter.hpp"
#include "ParameterRequest.hpp"
#include "ParameterStorage.hpp"
#include "cpm/ParticipantSingleton.hpp"

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

    std::atomic<bool> is_active;

public:
    ParameterServer(std::shared_ptr<ParameterStorage> _storage, bool init_active_value);

    void set_active_callback(bool active);
    void resend_param_callback(std::string name);
    
    std::shared_ptr<ParameterStorage> storage;
};