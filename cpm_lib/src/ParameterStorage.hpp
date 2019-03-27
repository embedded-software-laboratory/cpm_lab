#pragma once

/*
 * Singleton that receives and stores constants, e.g. for configuration
 * Is used by ParameterDistribution to get data for the user
 * domain_id, parameterTopic and publisherTopic can be set before the singleton is used
 */

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "Parameter.hpp"
#include "ParameterRequest.hpp"

#include "cpm/AsyncReader.hpp"
#include <dds/pub/ddspub.hpp>

class ParameterStorage {
public:
    static ParameterStorage& Instance();

    //Delete move and copy op
    ParameterStorage(ParameterStorage const&) = delete;
    ParameterStorage(ParameterStorage&&) = delete; 
    ParameterStorage& operator=(ParameterStorage const&) = delete;
    ParameterStorage& operator=(ParameterStorage &&) = delete;

    //Provide access similar to the interface
    bool parameter_bool(std::string parameter_name);
    int32_t parameter_int(std::string parameter_name);
    double parameter_double(std::string parameter_name);
    std::string parameter_string(std::string parameter_name);
    std::vector<int32_t> parameter_ints(std::string parameter_name);
    std::vector<double> parameter_doubles(std::string parameter_name);

private:
    ParameterStorage();

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

    void requestParam(std::string parameter_name);
    void callback(dds::sub::LoanedSamples<Parameter>& samples);

public:
    dds::topic::Topic<Parameter> parameterTopic;
    dds::topic::Topic<ParameterRequest> parameterRequestTopic;

private:
    dds::pub::DataWriter<ParameterRequest> writer;
    cpm::AsyncReader<Parameter> subscriber;
};
