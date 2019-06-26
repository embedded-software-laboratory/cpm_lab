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
            log_storage.push_back(sample.data());
        }
    }
}

std::vector<Log> LogStorage::get_new_logs() {
    std::lock_guard<std::mutex> lock(log_storage_mutex);
    std::vector<Log> log_copy = log_storage;
    log_storage.clear();
    return log_copy;
}