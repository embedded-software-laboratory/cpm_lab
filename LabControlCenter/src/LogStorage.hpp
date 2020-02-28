#pragma once

#include "defaults.hpp"
#include <atomic>
#include <cassert>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <glib.h>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "Log.hpp"

/**
 * \brief Used to receive and store Log messages (cpm::Logging) from all participants in the current domain
 */
class LogStorage {
public:
    enum FilterType {ID, Content, Timestamp, All};

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

    //File for logging
    std::ofstream file;
    std::string filename = "all_received_logs.csv"; 
    //Mutex s.t. only one thread has access to the file
    std::mutex file_mutex;

    //Clear elements so that count last elements are kept
    void keep_last_elements(std::vector<Log>& vector, size_t count);

    /**
     * \brief Gtk will complain about non-valid UTF-8 strings when added to the UI
     * Thus, this function appends a warning that the message is invalid to invalid log messages
     * The user thus finds out that some of his log messages of his program (-> node_id) are invalid
     * This function checks all parts of the message, but only changes the log message
     * Gtk will still show warnings for invalid messages (Pango), but the user, when looking at the logs,
     * should notice that his log messages are invalid, and where to find them (to be able to correct them)
     * 
     * Also affects invalid_utf8_detected: If true, no more logs are stored
     */
    void assert_utf8_validity(Log& log);
    std::atomic_bool invalid_utf8_detected; //If true: Storing logs is stopped (to make the error prominent)

public:
    LogStorage();
    ~LogStorage();

    std::vector<Log> get_new_logs();
    
    std::vector<Log> get_all_logs();

    /**
     * \brief Performs a search that is supposed to be run asynchronously in a new thread - using a future is recommended to obtain the result. The search can be aborted by setting continue_search to false (should thus be false at start) - this is useful in case the user starts a new search before the old one is completed
     * \param filter_value the string to search for (TODO: Later REGEX)
     * \param filter_type where the filter should match (Log message, Log ID...)
     * \param continue_search should be true initially, set to false to abort the search before it finished - the algorithm then returns immediately
     */
    std::vector<Log> perform_abortable_search(std::string filter_value, FilterType filter_type, std::atomic_bool &continue_search);

    /**
    * \brief Reset all data structures / delete all log data
    */
    void reset();
};