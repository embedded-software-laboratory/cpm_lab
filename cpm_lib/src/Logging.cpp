#include "cpm/Logging.hpp"

Logging::Logging() :
    loggingTopic(cpm::ParticipantSingleton::Instance(), "Logs"),
    logger(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), loggingTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
{
    file.open(filename, std::ofstream::out | std::ofstream::trunc);
    file.close();
}

Logging& Logging::Instance() {
    static Logging instance;
    return instance;
}

void Logging::flush() {
    file.open(filename, std::ios::app);
	file << stream.str() << "\n";
	file.close();

    ParameterRequest request(stream.str());
    logger.write(request);

    //Clear the stream
    stream.str(std::string());
}