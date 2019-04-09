#include "cpm/Logging.hpp"

Logging::Logging() :
    loggingTopic(cpm::get_topic<Log>(cpm::ParticipantSingleton::Instance(), "Logs")),
    logger(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), loggingTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
{

    // Formatted start time for log filename
    char time_format_buffer[100];
    {
        struct tm* tm_info;
        time_t timer;
        time(&timer);
        tm_info = gmtime(&timer);
        strftime(time_format_buffer, 100, "%Y_%m_%d_%H_%M_%S", tm_info);
    }


    filename = "Log_";
    filename += time_format_buffer;
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
    //Mutex bc value could be set by different threads at once
    std::lock_guard<std::mutex> lock(log_mutex);

    id = _id;
}

std::string Logging::get_filename() {
    return filename;
}

void Logging::check_id() {
    //Mutex bc value could be set by different threads at once (id could be set with set_id while it is read)
    std::lock_guard<std::mutex> lock(log_mutex);

    if (id == "uninitialized") {
        fprintf(stderr, "Error: Logger ID was never set\n");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
}