#pragma once

#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "LogLevel.hpp"

#include <memory>

/**
 * \class LogLevelSetter
 * \brief This Singleton is nothing more than a writer that allows to set the log level for all participants within the domain of the cpm library participant Singleton
 * It is used by UI tools, after a user interaction where the log level is modified
 * This class is currently only used in the 'Logs' Tab
 */

class LogLevelSetter {
private:
    //The writer is set to be transient local and reliable, s.t. a participant that joins the domain after the log level was set can still receive the last set value
    dds::pub::DataWriter<LogLevel> log_level_writer;

    /**
     * \brief The constructor sets up the DDS Writer
     */
    LogLevelSetter();

    //Delete all methods that are not Singleton-conform
    LogLevelSetter(LogLevelSetter const&) = delete;
    LogLevelSetter(LogLevelSetter&&) = delete; 
    LogLevelSetter& operator=(LogLevelSetter const&) = delete;
    LogLevelSetter& operator=(LogLevelSetter &&) = delete;

public:
    /**
     * \brief Provides access to the Singleton instance
     * \return The LogLevelSetter instance
     */
    static LogLevelSetter& Instance();

    /**
     * \brief This function is used to set the log level for the whole domain (of the cpm library participant Singleton)
     * \param level The desired level / verbosity of the logs (0: None, 1: Critical system failures, 2: Important (debug) information and 1, 3: Any logged information and 1,2)
     */
    void set_log_level(unsigned short level);
};