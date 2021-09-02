#pragma once

#include "defaults.hpp"
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

//For getting time in H:M:S
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <glib.h>

/**
 * \brief LCCErrorLogger is a Singleton class that is used throughout the LCC to log error messages that would else be shown in the console, which might not be directly related to the simulation
 * (For this reason, cpm::Logging is not used)
 * It is e.g. used in some CommonRoad drawing classes to log errors when draw() is called but some conditions are not fulfilled
 * \ingroup lcc
 */
class LCCErrorLogger {
    LCCErrorLogger(LCCErrorLogger const&) = delete;
    LCCErrorLogger(LCCErrorLogger&&) = delete; 
    LCCErrorLogger& operator=(LCCErrorLogger const&) = delete;
    LCCErrorLogger& operator=(LCCErrorLogger &&) = delete;

private:
    //! For already taken error messages (using get_new_errors). Unordered maps are used because we only want to show and store each error message (key) once. Error timestamps (value) may change if the same message gets emitted again
    std::unordered_map<std::string, std::string> error_storage;
    //! For new error messages that have not yet been taken (using get_new_errors). Unordered maps are used because we only want to show and store each error message (key) once. Error timestamps (value) may change if the same message gets emitted again
    std::unordered_map<std::string, std::string> new_error_storage;
    //! Mutex for error_storage
    std::mutex error_storage_mutex;
    //! Mutex for new_error_storage
    std::mutex new_error_storage_mutex;

    /**
     * \brief Constructor, made private s.t. singleton property is fulfilled
     */
    LCCErrorLogger() {};

    /**
     * \brief A simple function relying on std::chrono to get the current time in Hours:Minutes:Seconds
     */
    std::string get_timestamp_string();

public:
    /**
     * \brief Retrieve the participant singleton with this function
     * \return A participant
     */
    static LCCErrorLogger& Instance();

    /**
     * \brief Store the given error string and show it in the UI. Equal error strings are only stored once
     * \param error Error string / description
     */
    void log_error(std::string error);
    
    /**
     * \brief Get all LCC error messages that have been received
     * \return Vector of error messages
     */
    std::unordered_map<std::string, std::string> get_all_errors();

    /**
     * \brief Get all LCC error messages that have been received since the last request of new errors
     * \return Vector of error messages
     */
    std::unordered_map<std::string, std::string> get_new_errors();

    /**
    * \brief Reset all data structures / delete all error data
    */
    void reset();
};