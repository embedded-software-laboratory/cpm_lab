#include "parameter_distribution.hpp"

cpm::Parameter_distribution::Parameter_distribution() {

}

bool cpm::Parameter_distribution::parameter_bool(std::string parameter_name) {

}

int32_t cpm::Parameter_distribution::parameter_int(std::string parameter_name) {

}

double cpm::Parameter_distribution::parameter_double(std::string parameter_name) {

}

std::string cpm::Parameter_distribution::parameter_string(std::string parameter_name) {

}

std::vector<int32_t> cpm::Parameter_distribution::parameter_ints(std::string parameter_name) {

}

std::vector<double> cpm::Parameter_distribution::parameter_doubles(std::string parameter_name) {

}

std::vector<std::string> cpm::Parameter_distribution::parameter_strings(std::string parameter_name) {

}

void cpm::Parameter_distribution::requestParam(std::string parameter_name) {
    todo msg;
    publisher.send(msg);
}