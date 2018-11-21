#include "ParameterStorage.hpp"

int ParameterStorage::domain_id = 0; 
std::string ParameterStorage::subscriberTopicName = "parameter"; 
std::string ParameterStorage::publisherTopicName = "parameterRequest";

using namespace std::placeholders;

ParameterStorage::ParameterStorage() : 
    participant(domain_id),
    subscriberTopic(participant, subscriberTopicName),
    writerTopic(participant, publisherTopicName),
    writer(dds::pub::Publisher(participant), writerTopic),
    subscriber(subscriberTopicName, std::bind(&ParameterStorage::callback, this, _1), participant, subscriberTopic)
{
    
}

ParameterStorage& ParameterStorage::Instance() {
    // Thread-safe in C++11
    static ParameterStorage myInstance;
    return myInstance;
}

bool ParameterStorage::parameter_bool(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_bool_mutex); 

    while (param_bool.find(parameter_name) == param_bool.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    bool retValue = param_bool.at(parameter_name);
    s_lock.unlock();
    return retValue;
}

int32_t ParameterStorage::parameter_int(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_int_mutex); 

    while (param_int.find(parameter_name) == param_int.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    int32_t retValue = param_int.at(parameter_name);
    s_lock.unlock();
    return retValue;
}

double ParameterStorage::parameter_double(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_double_mutex); 

    while (param_double.find(parameter_name) == param_double.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    double retValue = param_double.at(parameter_name);
    s_lock.unlock();
    return retValue;
}

std::string ParameterStorage::parameter_string(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_string_mutex); 

    while (param_string.find(parameter_name) == param_string.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    std::string retValue = param_string.at(parameter_name);
    s_lock.unlock();
    return retValue;
}

std::vector<int32_t> ParameterStorage::parameter_ints(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_ints_mutex); 

    while (param_ints.find(parameter_name) == param_ints.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    std::vector<int32_t> retValue(param_ints.at(parameter_name));
    s_lock.unlock();
    return retValue;
}

std::vector<double> ParameterStorage::parameter_doubles(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_doubles_mutex); 

    while (param_doubles.find(parameter_name) == param_doubles.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    std::vector<double> retValue(param_doubles.at(parameter_name));
    s_lock.unlock();
    return retValue;
}

std::vector<std::string> ParameterStorage::parameter_strings(std::string parameter_name) {
    std::unique_lock<std::mutex> s_lock(param_strings_mutex); 

    while (param_strings.find(parameter_name) == param_strings.end()) {
        s_lock.unlock();
        requestParam(parameter_name);
        std::cout << "Waiting for parameter " << parameter_name << std::endl;
        rti::util::sleep(dds::core::Duration::from_millisecs(static_cast<uint64_t>(1000)));
        s_lock.lock();
    }

    std::vector<std::string> retValue(param_strings.at(parameter_name));
    s_lock.unlock();
    return retValue;
}

void ParameterStorage::requestParam(std::string parameter_name) {
    ParameterRequest request;
    request.name(parameter_name);
    writer.write(request);
}

void ParameterStorage::callback(dds::sub::LoanedSamples<Parameter>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            const auto& parameter = sample.data();
            if (parameter.type() == ParameterType::Int32) {
                if (parameter.values_int32().size() == 0) {
                    std::cout << "Received null param" << std::endl;
                } 
                else {
                    std::unique_lock<std::mutex> u_lock(param_int_mutex);
                    param_int.insert(std::pair<std::string, int32_t>(parameter.name(), parameter.values_int32()[0]));
                    u_lock.unlock();
                }
            }
            else if (parameter.type() == ParameterType::Double) {
                if (parameter.values_double().size() == 0) {
                    std::cout << "Received null param" << std::endl;
                } 
                else {
                    std::unique_lock<std::mutex> u_lock(param_double_mutex);
                    param_double.insert(std::pair<std::string, double>(parameter.name(), parameter.values_double()[0]));
                    u_lock.unlock();
                }
            }
            else if (parameter.type() == ParameterType::String) {
                if (parameter.values_string().size() == 0) {
                    std::cout << "Received null param" << std::endl;
                } 
                else {
                    std::unique_lock<std::mutex> u_lock(param_string_mutex);
                    param_string.insert(std::pair<std::string, std::string>(parameter.name(), parameter.values_string()[0]));
                    u_lock.unlock();
                }
            }
            else if (parameter.type() == ParameterType::Bool) {
                std::unique_lock<std::mutex> u_lock(param_bool_mutex);
                param_bool.insert(std::pair<std::string, bool>(parameter.name(), parameter.value_bool()));
                u_lock.unlock();
            }
            else if (parameter.type() == ParameterType::Vector_Int32) {
                if (parameter.values_int32().size() == 0) {
                    std::cout << "Received null param" << std::endl;
                } 
                else {
                    std::unique_lock<std::mutex> u_lock(param_ints_mutex);
                    param_ints.insert(std::pair<std::string, std::vector<int32_t>>(parameter.name(), std::vector<int32_t>(parameter.values_int32().begin(), parameter.values_int32().end())));
                    u_lock.unlock();
                }
            }
            else if (parameter.type() == ParameterType::Vector_Double) {
                if (parameter.values_double().size() == 0) {
                    std::cout << "Received null param" << std::endl;
                } 
                else {
                    std::unique_lock<std::mutex> u_lock(param_doubles_mutex);
                    param_doubles.insert(std::pair<std::string, std::vector<double>>(parameter.name(), std::vector<double>(parameter.values_double().begin(), parameter.values_double().end())));
                    u_lock.unlock();
                }
            }
            else if (parameter.type() == ParameterType::Vector_String) {
                if (parameter.values_int32().size() == 0) {
                    std::cout << "Received null param" << std::endl;
                } 
                else {
                    std::unique_lock<std::mutex> u_lock(param_strings_mutex);
                    param_strings.insert(std::pair<std::string, std::vector<std::string>>(parameter.name(), std::vector<std::string>(parameter.values_string().begin(), parameter.values_string().end())));
                    u_lock.unlock();
                }
            }
        }
    }
}