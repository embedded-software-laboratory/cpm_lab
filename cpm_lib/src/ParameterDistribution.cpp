#include "cpm/Parameter.hpp"
#include "ParameterReceiver.hpp"

bool cpm::parameter_bool(std::string parameter_name) {
    return ParameterReceiver::Instance().parameter_bool(parameter_name);
}

int32_t cpm::parameter_int(std::string parameter_name) {
    return ParameterReceiver::Instance().parameter_int(parameter_name);
}

double cpm::parameter_double(std::string parameter_name) {
    return ParameterReceiver::Instance().parameter_double(parameter_name);
}

std::string cpm::parameter_string(std::string parameter_name) {
    return ParameterReceiver::Instance().parameter_string(parameter_name);
}

std::vector<int32_t> cpm::parameter_ints(std::string parameter_name) {
    return ParameterReceiver::Instance().parameter_ints(parameter_name);
}

std::vector<double> cpm::parameter_doubles(std::string parameter_name) {
    return ParameterReceiver::Instance().parameter_doubles(parameter_name);
}