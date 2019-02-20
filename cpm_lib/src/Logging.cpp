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

void Logging::set_id(std::string _id) {
    id = _id;
}

void Logging::flush() {
    file.open(filename, std::ios::app);
	file << stream.str() << "\n";
	file.close();

    uint64_t time_now = 0;
    Log log(id, stream.str(), TimeStamp(0));
    logger.write(log);

    //Clear the stream
    stream.str(std::string());
}