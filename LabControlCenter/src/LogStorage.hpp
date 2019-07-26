#pragma once

#include "defaults.hpp"
#include <cassert>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "Log.hpp"

class LogStorage {
private:
    //Communication objects and callbacks
    void log_callback(dds::sub::LoanedSamples<Log>& samples);
    cpm::AsyncReader<Log> log_reader;
    //Only keeps the newest logs, used when not in search-mode
    std::vector<Log> log_buffer; //TODO choose a more useful data structure depending on what is supposed to be done with the Logs
    //Keeps all logs (might delete oldest ones if some limit is reached)
    std::vector<Log> log_storage;
    std::mutex log_buffer_mutex;
    std::mutex log_storage_mutex;

public:
    LogStorage();

    std::vector<Log> get_new_logs();
};