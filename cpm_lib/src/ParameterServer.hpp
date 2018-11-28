#pragma once

/*
 * ParameterServer.hpp
 * Server that distributes parameter values
*/

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "../build/rti/Parameter.hpp"
#include "../build/rti/ParameterRequest.hpp"

#include "Subscriber.hpp"
#include <dds/pub/ddspub.hpp>

class ParameterServer {
public:
    ParameterServer();

    void set_value(std::string name, bool value);
    void set_value(std::string name, int32_t value);
    void set_value(std::string name, double value);
    void set_value(std::string name, std::string value);
    void set_value(std::string name, std::vector<int32_t> value);
    void set_value(std::string name, std::vector<double> value);
    void set_value(std::string name, std::vector<std::string> value);
private:
    //Callback
    void handleParamRequest(dds::sub::LoanedSamples<ParameterRequest>& samples);

    //Variable storage, DDS request is sent only if the storage for key 'parameter_name' is empty
    std::map<std::string, bool> param_bool;
    std::map<std::string, int32_t> param_int;
    std::map<std::string, double> param_double;
    std::map<std::string, std::string> param_string;
    std::map<std::string, std::vector<int32_t>> param_ints;
    std::map<std::string, std::vector<double>> param_doubles;
    std::map<std::string, std::vector<std::string>> param_strings;

    //Mutex for each map
    std::mutex param_bool_mutex;
    std::mutex param_int_mutex;
    std::mutex param_double_mutex;
    std::mutex param_string_mutex;
    std::mutex param_ints_mutex;
    std::mutex param_doubles_mutex;
    std::mutex param_strings_mutex;

    dds::domain::DomainParticipant participant;
    dds::topic::Topic<ParameterRequest> subscriberTopic;
    dds::topic::Topic<Parameter> writerTopic;
    dds::pub::DataWriter<Parameter> writer;
    Subscriber<ParameterRequest> subscriber;
};