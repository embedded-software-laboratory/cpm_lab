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

uint64_t Logging::get_time() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

void Logging::set_id(std::string _id) {
    id = _id;
}

void Logging::flush() {
    file.open(filename, std::ios::app);
	file << stream.str() << "\n";
	file.close();

    uint64_t time_now = 0;
    Log log(id, stream.str(), TimeStamp(get_time()));
    logger.write(log);

    //Clear the stream
    stream.str(std::string());
}