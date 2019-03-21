#include "cpm/Logging.hpp"

Logging::Logging() :
    loggingTopic(cpm::ParticipantSingleton::Instance(), "Logs"),
    logger(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), loggingTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
{
    filename = "Log_";
    filename += std::to_string(get_time());
    filename += ".csv";

    file.open(filename, std::ofstream::out | std::ofstream::trunc);
    file << "ID,Timestamp,Content" << std::endl;
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

std::string Logging::get_filename() {
    return filename;
}

void Logging::flush() {
    check_id();
    
    uint64_t time_now = get_time();

    file.open(filename, std::ios::app);
	file << id << "," << time_now << "," << stream.str() << std::endl;
	file.close();

    Log log(id, stream.str(), TimeStamp(time_now));
    logger.write(log);

    std::cerr << "Log at time " << time_now << ": " << stream.str() << std::endl;

    //Clear the stream
    stream.str(std::string());
    stream.clear();
}

void Logging::check_id() {
    if (id == "uninitialized") {
        fprintf(stderr, "Error: Logger ID was never set\n");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
}