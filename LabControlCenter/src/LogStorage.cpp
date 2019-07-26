#include "LogStorage.hpp"

using namespace std::placeholders;
LogStorage::LogStorage() :
    /*Set up communication*/
    log_reader(std::bind(&LogStorage::log_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<Log>("Logs"), true)
{    
}

void LogStorage::log_callback(dds::sub::LoanedSamples<Log>& samples) {    
    for (auto sample : samples) {
        if (sample.info().valid()) {
            std::lock_guard<std::mutex> lock(log_storage_mutex);
            std::lock_guard<std::mutex> lock(log_buffer_mutex);
            log_storage.push_back(sample.data());
            log_buffer.push_back(sample.data());
        }
    }

    //TODO: Clear storage and buffer when some max size was reached
}

std::vector<Log> LogStorage::get_new_logs() {
    std::lock_guard<std::mutex> lock(log_buffer_mutex);
    std::vector<Log> log_copy = log_buffer;
    log_buffer.clear();
    return log_copy;
}