#include "cpm/Logging.hpp"

Logging::Logging() :
    loggingTopic(cpm::ParticipantSingleton::Instance(), "Logs"),
    logger(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), loggingTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
{

}

Logging& Logging::Instance() {
    static Logging instance;
    return instance;
}

void Logging::log(std::string msg) {
    ParameterRequest request(msg);
    logger.write(request);
}

template<typename T> Logging& Logging::operator<< (const T& log) {
    std::stringstream sstream;
    sstream << log;

    file.open(filename, std::ios::app);
	file << log << "\n";
	file.close();

    ParameterRequest request(sstream.str());
    logger.write(request);
}