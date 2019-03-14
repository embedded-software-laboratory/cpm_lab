#include "ParameterStorage.hpp"

ParameterStorage::ParameterStorage() {

}

ParameterStorage& ParameterStorage::Instance() {
    // Thread-safe in C++11
    static ParameterStorage myInstance;
    return myInstance;
}

void ParameterStorage::loadFile() {

}

void ParameterStorage::storeFile() {

}

void ParameterStorage::set_parameter_bool(std::string name, bool value) {
    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
    param_bool[name] = value;
}
void ParameterStorage::set_parameter_int(std::string name, int32_t value) {
    std::lock_guard<std::mutex> u_lock(param_int_mutex);
    param_int[name] = value;
}
void ParameterStorage::set_parameter_double(std::string name, double value) {
    std::lock_guard<std::mutex> u_lock(param_double_mutex);
    param_double[name] = value;
}
void ParameterStorage::set_parameter_string(std::string name, std::string value) {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    param_string[name] = value;
}
void ParameterStorage::set_parameter_string(std::string name, const char* value) {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    param_string[name] = std::string(value);
}
void ParameterStorage::set_parameter_ints(std::string name, std::vector<int32_t> value) {
    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
    param_ints[name] = value;
}
void ParameterStorage::set_parameter_doubles(std::string name, std::vector<double> value) {
    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
    param_doubles[name] = value;
}

bool ParameterStorage::get_parameter_bool(std::string name, bool& value) {
    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
    if (param_bool.find(name) != param_bool.end()) {
        value = param_bool[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_int(std::string name, int32_t& value) {
    std::lock_guard<std::mutex> u_lock(param_int_mutex);
    if (param_int.find(name) != param_int.end()) {
        value = param_int[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_double(std::string name, double& value) {
    std::lock_guard<std::mutex> u_lock(param_double_mutex);
    if (param_double.find(name) != param_double.end()) {
        value = param_double[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_string(std::string name, std::string& value) {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    if (param_string.find(name) != param_string.end()) {
        value = param_string[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_ints(std::string name, std::vector<int32_t>& value) {
    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
    if (param_ints.find(name) != param_ints.end()) {
        value = param_ints[name];
        return true;
    }
    return false;
}
bool ParameterStorage::get_parameter_doubles(std::string name, std::vector<double>& value) {
    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
    if (param_doubles.find(name) != param_doubles.end()) {
        value = param_doubles[name];
        return true;
    }
    return false;
}

std::vector<std::string> ParameterStorage::list_bool() {
    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_bool) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_int() {
    std::lock_guard<std::mutex> u_lock(param_int_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_int) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_double() {
    std::lock_guard<std::mutex> u_lock(param_double_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_double) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_string() {
    std::lock_guard<std::mutex> u_lock(param_string_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_string) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_ints() {
    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_ints) {
        param_names.push_back(entry.first);
    }
    return param_names;
}
std::vector<std::string> ParameterStorage::list_doubles() {
    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
    std::vector<std::string> param_names;
    for (auto const& entry : param_doubles) {
        param_names.push_back(entry.first);
    }
    return param_names;
}