#include "ParameterStorage.hpp"

ParameterStorage::ParameterStorage(int domain_id, std::string subscriberTopicName, std::string publisherTopicName) : 
    participant(domain_id),
	subscriberTopic(participant, subscriberTopicName),
    writerTopic(participant, publisherTopicName),
    writer(dds::pub::Publisher(participant), writerTopic),
    subscriber()
{
    
}

ParameterStorage& ParameterStorage::Instance(int domain_id, std::string subscriberTopic, std::string publisherTopic) {
    // Thread-safe in C++11
        static ParameterStorage myInstance(domain_id, subscriberTopic, publisherTopic);

        return myInstance;
}

bool ParameterStorage::parameter_bool(std::string parameter_name) {

}

int32_t ParameterStorage::parameter_int(std::string parameter_name) {

}

double ParameterStorage::parameter_double(std::string parameter_name) {

}

std::string ParameterStorage::parameter_string(std::string parameter_name) {

}

std::vector<int32_t> ParameterStorage::parameter_ints(std::string parameter_name) {

}

std::vector<double> ParameterStorage::parameter_doubles(std::string parameter_name) {

}

std::vector<std::string> ParameterStorage::parameter_strings(std::string parameter_name) {

}

void ParameterStorage::requestParam(std::string parameter:name) {
    ParameterRequest request;
    request.name(parameter_name);
    writer.write(request);
}

void ParameterStorage::callback(dds::sub::LoanedSamples<VehicleCommandCurvature>& samples) {

}