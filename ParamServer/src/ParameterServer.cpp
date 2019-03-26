#include "ParameterServer.hpp"

using namespace std::placeholders;

ParameterServer::ParameterServer(ParameterStorage& _storage):
    parameterTopic(cpm::ParticipantSingleton::Instance(), "parameter"),
    parameterRequestTopic(cpm::ParticipantSingleton::Instance(), "parameterRequest"),
    writer(
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        parameterTopic,
        dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()
    ),
    subscriber(
        "parameterRequest", 
        std::bind(&ParameterServer::handleParamRequest, this, _1), 
        cpm::ParticipantSingleton::Instance(), 
        parameterRequestTopic
    ),
    storage(_storage)
{
}

void ParameterServer::handleParamRequest(dds::sub::LoanedSamples<ParameterRequest>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            handleSingleParamRequest(sample.data().name());
        }
    }
}

void ParameterServer::handleSingleParamRequest(std::string name) {
    std::cout << "Got request: " << name << std::endl;

    Parameter param = Parameter();
    param.name(name);

    bool boolParam;
    if(storage.get_parameter_bool(name, boolParam)) {
        param.type(ParameterType::Bool);
        param.value_bool(boolParam);

        writer.write(param);
        return;
    }

    int32_t intParam;
    if(storage.get_parameter_int(name, intParam)) {
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
    if(storage.get_parameter_double(name, doubleParam)) {
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
    if(storage.get_parameter_string(name, stringParam)) {
        param.type(ParameterType::String);
        param.value_string(stringParam);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::vector<int32_t> intParams;
    if(storage.get_parameter_ints(name, intParams)) {
        //Create data to send
        rti::core::vector<int32_t> ints(intParams);

        param.type(ParameterType::Vector_Int32);
        param.values_int32(ints);
        
        //Send new value
        writer.write(param);
        return;
    }

    std::vector<double> doubleParams;
    if(storage.get_parameter_doubles(name, doubleParams)) {
        //Create data to send
        rti::core::vector<double> doubles(doubleParams);

        param.type(ParameterType::Vector_Double);
        param.values_double(doubles);
        
        //Send new value
        writer.write(param);
        return;
    }
}