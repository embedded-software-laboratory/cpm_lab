#include "cpm/ParameterReceiver.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/get_topic.hpp"

using namespace std::placeholders;


namespace cpm
{

    bool parameter_bool(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_bool(parameter_name);
    }

    int32_t parameter_int(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_int(parameter_name);
    }

    double parameter_double(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_double(parameter_name);
    }

    std::string parameter_string(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_string(parameter_name);
    }

    std::vector<int32_t> parameter_ints(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_ints(parameter_name);
    }

    std::vector<double> parameter_doubles(std::string parameter_name) {
        return ParameterReceiver::Instance().parameter_doubles(parameter_name);
    }

    ParameterReceiver::ParameterReceiver():
        parameterTopic(cpm::get_topic<Parameter>("parameter")),
        parameterRequestTopic(cpm::get_topic<ParameterRequest>("parameterRequest")),
        writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), parameterRequestTopic),
        subscriber(std::bind(&ParameterReceiver::callback, this, _1), cpm::ParticipantSingleton::Instance(), parameterTopic)
    {

    }

    ParameterReceiver& ParameterReceiver::Instance() {
        // Thread-safe in C++11
        static ParameterReceiver myInstance;
        return myInstance;
    }

    bool ParameterReceiver::parameter_bool(std::string parameter_name) {
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

    int32_t ParameterReceiver::parameter_int(std::string parameter_name) {
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

    double ParameterReceiver::parameter_double(std::string parameter_name) {
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

    std::string ParameterReceiver::parameter_string(std::string parameter_name) {
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

    std::vector<int32_t> ParameterReceiver::parameter_ints(std::string parameter_name) {
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

    std::vector<double> ParameterReceiver::parameter_doubles(std::string parameter_name) {
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

    void ParameterReceiver::requestParam(std::string parameter_name) {
        ParameterRequest request;
        request.name(parameter_name);
        writer.write(request);
    }

    void ParameterReceiver::callback(dds::sub::LoanedSamples<Parameter>& samples) {
        for (auto sample : samples) {
            if (sample.info().valid()) {
                const auto& parameter = sample.data();

                if (parameter.type() == ParameterType::Int32 && parameter.values_int32().size() == 1) {
                    std::lock_guard<std::mutex> u_lock(param_int_mutex);
                    param_int[parameter.name()] = parameter.values_int32().at(0);
                }
                else if (parameter.type() == ParameterType::Double && parameter.values_double().size() == 1) {
                    std::lock_guard<std::mutex> u_lock(param_double_mutex);
                    param_double[parameter.name()] = parameter.values_double().at(0);
                }
                else if (parameter.type() == ParameterType::String) {
                    std::lock_guard<std::mutex> u_lock(param_string_mutex);
                    param_string[parameter.name()] = parameter.value_string();
                }
                else if (parameter.type() == ParameterType::Bool) {
                    std::lock_guard<std::mutex> u_lock(param_bool_mutex);
                    param_bool[parameter.name()] = parameter.value_bool();
                }
                else if (parameter.type() == ParameterType::Vector_Int32) {
                    std::lock_guard<std::mutex> u_lock(param_ints_mutex);
                    param_ints[parameter.name()] = parameter.values_int32();
                }
                else if (parameter.type() == ParameterType::Vector_Double) {
                    std::lock_guard<std::mutex> u_lock(param_doubles_mutex);
                    param_doubles[parameter.name()] = parameter.values_double();
                }
            }
        }
    }
}