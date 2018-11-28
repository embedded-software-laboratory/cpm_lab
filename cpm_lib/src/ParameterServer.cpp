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

    //Create data to send
    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::Bool);
    param.value_bool(value);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::set_value(std::string name, int32_t value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_int_mutex); 
    param_int.insert(std::pair<std::string, int32_t>(name, value));
    s_lock.unlock();

    //Create data to send
    std::vector<int32_t> stdInts;
    stdInts.push_back(value);
    rti::core::vector<int32_t> ints = rti::core::vector<int32_t>(stdInts);

    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::Int32);
    param.values_int32(ints);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::set_value(std::string name, double value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_double_mutex); 
    param_double.insert(std::pair<std::string, double>(name, value));
    s_lock.unlock();

    //Create data to send
    std::vector<double> stdDoubles;
    stdDoubles.push_back(value);
    rti::core::vector<double> doubles(stdDoubles);

    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::Double);
    param.values_double(doubles);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::set_value(std::string name, std::string value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_string_mutex); 
    param_string.insert(std::pair<std::string, std::string>(name, value));
    s_lock.unlock();

    //Create data to send
    std::vector<std::string> stdStrings;
    stdStrings.push_back(value);
    rti::core::vector<dds::core::string> strings;
    std::copy(strings.begin(), strings.end(), std::back_inserter(stdStrings));

    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::String);
    param.values_string(strings);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::set_value(std::string name, std::vector<int32_t> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_ints_mutex); 
    param_ints.insert(std::pair<std::string, std::vector<int32_t>>(name, value));
    s_lock.unlock();

    //Create data to send
    rti::core::vector<int32_t> ints = rti::core::vector<int32_t>(value);

    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::Vector_Int32);
    param.values_int32(ints);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::set_value(std::string name, std::vector<double> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_doubles_mutex); 
    param_doubles.insert(std::pair<std::string, std::vector<double>>(name, value));
    s_lock.unlock();

    //Create data to send
    rti::core::vector<double> doubles(value);

    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::Vector_Double);
    param.values_double(doubles);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::set_value(std::string name, std::vector<std::string> value) {
    //Store new value
    std::unique_lock<std::mutex> s_lock(param_strings_mutex); 
    param_strings.insert(std::pair<std::string, std::vector<std::string>>(name, value));
    s_lock.unlock();

    //Create data to send
    rti::core::vector<dds::core::string> strings;
    std::copy(strings.begin(), strings.end(), std::back_inserter(value));

    Parameter param = Parameter();
    param.name(name);
    param.type(ParameterType::Vector_String);
    param.values_string(strings);
    
    //Send new value
    writer.write(param);
}

void ParameterServer::handleParamRequest(dds::sub::LoanedSamples<ParameterRequest>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            std::string name = sample.data().name();

            //Go through all maps, return if param found
            std::unique_lock<std::mutex> b_lock(param_bool_mutex);
            if(param_bool.find(name) != param_bool.end()) {
                //Create data to send
                Parameter param = Parameter();
                param.name(name);
                param.type(ParameterType::Bool);
                param.value_bool(param_bool.at(name));

                b_lock.unlock();
                
                //Send new value
                writer.write(param);
            }
            else {
                b_lock.unlock();

                std::unique_lock<std::mutex> i_lock(param_int_mutex);
                if(param_int.find(name) != param_int.end()) {
                    //Create data to send
                    std::vector<int32_t> stdInts;
                    stdInts.push_back(param_int.at(name));
                    i_lock.unlock();
                    rti::core::vector<int32_t> ints = rti::core::vector<int32_t>(stdInts);

                    Parameter param = Parameter();
                    param.name(name);
                    param.type(ParameterType::Int32);
                    param.values_int32(ints);
                    
                    //Send new value
                    writer.write(param);
                }
                else {
                    i_lock.unlock();

                    std::unique_lock<std::mutex> d_lock(param_double_mutex);
                    if(param_double.find(name) != param_double.end()) {
                        //Create data to send
                        std::vector<double> stdDoubles;
                        stdDoubles.push_back(param_double.at(name));
                        d_lock.unlock();
                        rti::core::vector<double> doubles(stdDoubles);

                        Parameter param = Parameter();
                        param.name(name);
                        param.type(ParameterType::Double);
                        param.values_double(doubles);
                        
                        //Send new value
                        writer.write(param);
                    }
                    else {
                        d_lock.unlock();

                        std::unique_lock<std::mutex> s_lock(param_string_mutex);
                        if(param_string.find(name) != param_string.end()) {
                            //Create data to send
                            std::vector<std::string> stdStrings;
                            stdStrings.push_back(param_string.at(name));
                            s_lock.unlock();
                            rti::core::vector<dds::core::string> strings;
                            std::copy(strings.begin(), strings.end(), std::back_inserter(stdStrings));

                            Parameter param = Parameter();
                            param.name(name);
                            param.type(ParameterType::String);
                            param.values_string(strings);
                            
                            //Send new value
                            writer.write(param);
                        }
                        else {
                            s_lock.unlock();

                            std::unique_lock<std::mutex> is_lock(param_ints_mutex);
                            if(param_ints.find(name) != param_ints.end()) {
                                //Create data to send
                                rti::core::vector<int32_t> ints = rti::core::vector<int32_t>(param_ints.at(name));
                                is_lock.unlock();

                                Parameter param = Parameter();
                                param.name(name);
                                param.type(ParameterType::Vector_Int32);
                                param.values_int32(ints);
                                
                                //Send new value
                                writer.write(param);
                            }
                            else {
                                is_lock.unlock();

                                std::unique_lock<std::mutex> ds_lock(param_doubles_mutex);
                                if(param_doubles.find(name) != param_doubles.end()) {
                                    //Create data to send
                                    rti::core::vector<double> doubles(param_doubles.at(name));
                                    ds_lock.unlock();

                                    Parameter param = Parameter();
                                    param.name(name);
                                    param.type(ParameterType::Vector_Double);
                                    param.values_double(doubles);
                                    
                                    //Send new value
                                    writer.write(param);
                                }
                                else {
                                    ds_lock.unlock();

                                    std::unique_lock<std::mutex> st_lock(param_strings_mutex);
                                    if(param_strings.find(name) != param_strings.end()) {
                                        //Create data to send
                                        rti::core::vector<dds::core::string> strings;
                                        std::copy(strings.begin(), strings.end(), std::back_inserter(param_strings.at(name)));
                                        st_lock.unlock();

                                        Parameter param = Parameter();
                                        param.name(name);
                                        param.type(ParameterType::Vector_String);
                                        param.values_string(strings);
                                        
                                        //Send new value
                                        writer.write(param);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}