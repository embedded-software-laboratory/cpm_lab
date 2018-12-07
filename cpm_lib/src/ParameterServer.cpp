#include "ParameterServer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "ParameterStorage.hpp"

using namespace std::placeholders;

ParameterServer::ParameterServer():
    writer(
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        ParameterStorage::Instance().parameterTopic
    ),
    subscriber(
        "parameterRequest", 
        std::bind(&ParameterServer::handleParamRequest, this, _1), 
        cpm::ParticipantSingleton::Instance(), 
        ParameterStorage::Instance().parameterRequestTopic
    )
{

}

void ParameterServer::set_value(std::string name, bool value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_bool_mutex); 
    param_bool[name] = value;
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, int32_t value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_int_mutex); 
    param_int[name] = value;
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, double value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_double_mutex); 
    param_double[name] = value;
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::string value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_string_mutex); 
    param_string[name] = value;
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::vector<int32_t> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_ints_mutex); 
    param_ints[name] = value;
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::vector<double> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_doubles_mutex); 
    param_doubles[name] = value;
    s_lock.unlock();

    //Fake request to send data
    handleSingleParamRequest(name);
}

void ParameterServer::set_value(std::string name, std::vector<std::string> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_strings_mutex); 
    param_strings[name] = value;
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
}

void ParameterServer::handleSingleParamRequest(std::string name) {
    Parameter param = Parameter();
    param.name(name);

    bool boolParam;
    if(find_bool(name, boolParam)) {
        param.type(ParameterType::Bool);
        param.value_bool(boolParam);

        writer.write(param);
        return;
    }

    int32_t intParam;
    if(find_int(name, intParam)) {
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

    double doubleParam;
    if(find_double(name, doubleParam)) {
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

    std::string stringParam;
    if(find_string(name, stringParam)) {
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

    std::vector<int32_t> intParams;
    if(find_ints(name, intParams)) {
        //Create data to send
        rti::core::vector<int32_t> ints(intParams);

        param.type(ParameterType::Vector_Int32);
        param.values_int32(ints);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::vector<double> doubleParams;
    if(find_doubles(name, doubleParams)) {
        //Create data to send
        rti::core::vector<double> doubles(doubleParams);

        param.type(ParameterType::Vector_Double);
        param.values_double(doubles);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::vector<std::string> stringParams;
    if(find_strings(name, stringParams)) {
        param.type(ParameterType::Vector_String);
        param.values_string().resize(stringParams.size());
        for (size_t i = 0; i < stringParams.size(); ++i) {
            param.values_string().at(i) = stringParams.at(i);
        }

        //Send new value
        writer.write(param);
        return;
    }
}

bool ParameterServer::find_bool(std::string param_name, bool &value_out) {
    std::lock_guard<std::mutex> lock(param_bool_mutex);
    if(param_bool.find(param_name) != param_bool.end()) {
        value_out = param_bool.at(param_name);
        return true;
    }
    return false;
}

bool ParameterServer::find_int(std::string param_name, int32_t &value_out) {
    std::lock_guard<std::mutex> lock(param_int_mutex);
    if(param_int.find(param_name) != param_int.end()) {
        value_out = param_int.at(param_name);
        return true;
    }
    return false;
}

bool ParameterServer::find_double(std::string param_name, double &value_out) {
    std::lock_guard<std::mutex> lock(param_double_mutex);
    if(param_double.find(param_name) != param_double.end()) {
        value_out = param_double.at(param_name);
        return true;
    }
    return false;
}

bool ParameterServer::find_string(std::string param_name, std::string &value_out) {
    std::lock_guard<std::mutex> lock(param_string_mutex);
    if(param_string.find(param_name) != param_string.end()) {
        value_out = param_string.at(param_name);
        return true;
    }
    return false;
}

bool ParameterServer::find_ints(std::string param_name, std::vector<int32_t> &value_out) {
    std::lock_guard<std::mutex> lock(param_ints_mutex);
    if(param_ints.find(param_name) != param_ints.end()) {
        value_out = param_ints.at(param_name);
        return true;
    }
    return false;
}

bool ParameterServer::find_doubles(std::string param_name, std::vector<double> &value_out) {
    std::lock_guard<std::mutex> lock(param_doubles_mutex);
    if(param_doubles.find(param_name) != param_doubles.end()) {
        value_out = param_doubles.at(param_name);
        return true;
    }
    return false;
}

bool ParameterServer::find_strings(std::string param_name, std::vector<std::string> &value_out) {
    std::lock_guard<std::mutex> lock(param_strings_mutex);
    if(param_strings.find(param_name) != param_strings.end()) {
        value_out = param_strings.at(param_name);
        return true;
    }
    return false;
}