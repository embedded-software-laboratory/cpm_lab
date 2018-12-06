#include "ParameterServer.hpp"

using namespace std::placeholders;

ParameterServer::ParameterServer():
    participant(0),
    subscriberTopic(participant, "parameterRequest"),
    writerTopic(participant, "parameter"),
    writer(dds::pub::Publisher(participant), writerTopic),
    subscriber("parameterRequest", std::bind(&ParameterServer::handleParamRequest, this, _1), participant, subscriberTopic)
{

}

void ParameterServer::set_value(std::string name, bool value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_bool_mutex); 
    param_bool.insert(std::pair<std::string, bool>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, int32_t value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_int_mutex); 
    param_int.insert(std::pair<std::string, int32_t>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, double value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_double_mutex); 
    param_double.insert(std::pair<std::string, double>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::string value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_string_mutex); 
    param_string.insert(std::pair<std::string, std::string>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::vector<int32_t> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_ints_mutex); 
    param_ints.insert(std::pair<std::string, std::vector<int32_t>>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::vector<double> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_doubles_mutex); 
    param_doubles.insert(std::pair<std::string, std::vector<double>>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::vector<std::string> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_strings_mutex); 
    param_strings.insert(std::pair<std::string, std::vector<std::string>>(name, value));
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::handleParamRequest(dds::sub::LoanedSamples<ParameterRequest>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            handleSingleParamRequest(sample.data().name());
        }
    }

    handleSingleParamRequest(std::string name) {
            Parameter param = Parameter();
            param.name(name);

            std::experimental::optional<bool> boolParam = find_bool(name);
            if (boolParam) {
                param.type(ParameterType::Bool);
                param.value_bool(boolParam);

                writer.write(param);
                return;
            }

            std::experimental::optional<int_32t> intParam = find_int(name);
            if (intParam) {
                //Create data to send
                std::vector<int32_t> stdInts;
                stdInts.push_back(intParam);
                rti::core::vector<int32_t> ints(stdInts);

                param.type(ParameterType::Int32);
                param.values_int32(ints);
                
                //Send new value
                writer.write(param);
                return;
            }

            std::experimental::optional<double> doubleParam = find_double(name);
            if (doubleParam) {
                //Create data to send
                std::vector<double> stdDoubles;
                stdDoubles.push_back(doubleParam);
                rti::core::vector<double> doubles(stdDoubles);

                param.type(ParameterType::Double);
                param.values_double(doubles);
                
                //Send new value
                writer.write(param);
                return;
            }

            std::experimental::optional<string> stringParam = find_string(name);
            if (doubleParam) {
                //Create data to send
                std::vector<std::string> stdStrings;
                stdStrings.push_back(stringParam);
                rti::core::vector<dds::core::string> strings;
                std::copy(strings.begin(), strings.end(), std::back_inserter(stdStrings));

                param.type(ParameterType::String);
                param.values_string(strings);
                
                //Send new value
                writer.write(param);
                return;
            }

            std::experimental::optional<int_32t> intParams = find_ints(name);
            if (intParams) {
                //Create data to send
                rti::core::vector<int32_t> ints(intParams);

                param.type(ParameterType::Vector_Int32);
                param.values_int32(ints);
                
                //Send new value
                writer.write(param);
                return;
            }

            std::experimental::optional<double> doubleParams = find_doubles(name);
            if (doubleParams) {
                //Create data to send
                rti::core::vector<double> doubles(doubleParams);

                param.type(ParameterType::Vector_Double);
                param.values_double(doubles);
                
                //Send new value
                writer.write(param);
                return;
            }

            std::experimental::optional<string> stringParams = find_strings(name);
            if (stringParams) {
                //Create data to send
                rti::core::vector<dds::core::string> strings;
                std::copy(strings.begin(), strings.end(), std::back_inserter(stringParams));

                param.type(ParameterType::Vector_String);
                param.values_string(strings);
                
                //Send new value
                writer.write(param);
                return;
            }
    }

    std::experimental::optional<bool> ParameterServer::find_bool(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_bool_mutex);
        if(param_bool.find(name) != param_bool.end()) {
            return param_bool.at(name);
        }
    }

    std::experimental::optional<int32_t> ParameterServer::find_int(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_int_mutex);
        if(param_int.find(name) != param_int.end()) {
            return param_int.at(name);
        }
    }

    std::experimental::optional<double> ParameterServer::find_double(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_double_mutex);
        if(param_double.find(name) != param_double.end()) {
            return param_double.at(name);
        }
    }

    std::experimental::optional<std::string> ParameterServer::find_string(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_string_mutex);
        if(param_string.find(name) != param_string.end()) {
            return param_string.at(name);
        }
    }

    std::experimental::optional<std::vector<int32_t>> ParameterServer::find_ints(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_ints_mutex);
        if(param_ints.find(name) != param_ints.end()) {
            return param_ints.at(name);
        }
    }

    std::experimental::optional<std::vector<double>> ParameterServer::find_doubles(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_doubles_mutex);
        if(param_doubles.find(name) != param_doubles.end()) {
            return param_doubles.at(name);
        }
    }

    std::experimental::optional<std::vector<std::string>> ParameterServer::find_strings(std::string param_name) {
        std::lock_guard<std::mutex> lock(param_strings_mutex);
        if(param_strings.find(name) != param_strings.end()) {
            return param_strings.at(name);
        }
    }
}