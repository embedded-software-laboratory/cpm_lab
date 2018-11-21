#include "cpm/Parameter.hpp"

void cpm::init_param_distribution(int dom_id, std::string subscriberTopicName, std::string publisherTopicName) {
    ParameterStorage::domain_id = dom_id;
    ParameterStorage::subscriberTopicName = subscriberTopicName;
    ParameterStorage::publisherTopicName = publisherTopicName;
    ParameterStorage::Instance();
}

bool cpm::parameter_bool(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_bool(parameter_name);
}

int32_t cpm::parameter_int(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_int(parameter_name);
}

double cpm::parameter_double(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_double(parameter_name);
}

std::string cpm::parameter_string(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_string(parameter_name);
}

std::vector<int32_t> cpm::parameter_ints(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_ints(parameter_name);
}

std::vector<double> cpm::parameter_doubles(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_doubles(parameter_name);
}

std::vector<std::string> cpm::parameter_strings(std::string parameter_name) {
    return ParameterStorage::Instance().parameter_strings(parameter_name);
}  