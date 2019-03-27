#pragma once

/*
 * ParameterServer.hpp
 * Server that distributes parameter values. This server uses ParameterStorage to store its values.
*/

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "Parameter.hpp"
#include "ParameterRequest.hpp"
#include "ParameterStorage.hpp"
#include "cpm/ParticipantSingleton.hpp"

#include "cpm/AsyncReader.hpp"
#include <dds/pub/ddspub.hpp>

class ParameterServer {
public:
    ParameterServer(ParameterStorage& _storage);

private:    
    //Callback
    void handleParamRequest(dds::sub::LoanedSamples<ParameterRequest>& samples);
    void handleSingleParamRequest(std::string name);

    //Communication
    dds::topic::Topic<ParameterRequest> parameterRequestTopic;
    dds::topic::Topic<Parameter> parameterTopic;
    dds::pub::DataWriter<Parameter> writer;
    cpm::AsyncReader<ParameterRequest> readerParameterRequest;

    //Storage
    ParameterStorage& storage;
};