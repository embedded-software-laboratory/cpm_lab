#pragma once

/*
 *  ParameterDistribution.hpp
 *  This interface can be used to receive/request constant definitions of different types
 */

#include <string>
#include <vector>

#include "ParameterStorage.hpp"

namespace cpm {
    //Getter for the variables & initializer
    void init_param_distribution(int domain_id, std::string subscriberTopicName, std::string publisherTopicName);

    bool parameter_bool(std::string parameter_name);
    int32_t parameter_int(std::string parameter_name);
    double parameter_double(std::string parameter_name);
    std::string parameter_string(std::string parameter_name);
    std::vector<int32_t> parameter_ints(std::string parameter_name);
    std::vector<double> parameter_doubles(std::string parameter_name);
    std::vector<std::string> parameter_strings(std::string parameter_name);
}