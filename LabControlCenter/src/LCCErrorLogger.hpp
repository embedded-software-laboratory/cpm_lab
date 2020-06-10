#pragma once

#include "defaults.hpp"
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <glib.h>

/**
 * \brief LCCErrorLogger is a Singleton class that is used throughout the LCC to log error messages that would else be shown in the console, which might not be directly related to the simulation
 * (For this reason, cpm::Logging is not used)
 * It is e.g. used in some CommonRoad drawing classes to log errors when draw() is called but some conditions are not fulfilled
 */
class LCCErrorLogger {
    LCCErrorLogger(LCCErrorLogger const&) = delete;
    LCCErrorLogger(LCCErrorLogger&&) = delete; 
    LCCErrorLogger& operator=(LCCErrorLogger const&) = delete;
    LCCErrorLogger& operator=(LCCErrorLogger &&) = delete;

private:
    std::unordered_set<std::string> error_storage; //For already requested error messages
    std::unordered_set<std::string> new_error_storage; //For new error messages that have not yet been requested
    std::mutex error_storage_mutex;
    std::mutex new_error_storage_mutex;

    //Made private s.t. singleton property is fulfilled
    LCCErrorLogger() {};

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
    std::unordered_set<std::string> get_all_errors();

    /**
     * \brief Get all LCC error messages that have been received since the last request of new errors
     * \return Vector of error messages
     */
    std::unordered_set<std::string> get_new_errors();

    /**
    * \brief Reset all data structures / delete all error data
    */
    void reset();
};