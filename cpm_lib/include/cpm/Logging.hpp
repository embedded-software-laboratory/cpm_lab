#pragma once

/**
 * \class Logging.hpp
 * \brief This class can be used to log all relevant information or errors during runtime. These information are transmitted to the LabControlCenter.
 */

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <mutex>
#include <algorithm>

#include <dds/domain/DomainParticipant.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/dds.hpp>
#include <dds/core/ddscore.hpp>

#include "Log.hpp"

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

class Logging {
    Logging(Logging const&) = delete;
    Logging(Logging&&) = delete; 
    Logging& operator=(Logging const&) = delete;
    Logging& operator=(Logging &&) = delete;

    private:
        //DDS Writer for Logging
        dds::topic::Topic<Log> loggingTopic;
        dds::pub::DataWriter<Log> logger;

        //File for logging
        std::ofstream file;
        std::string filename = ""; //Is changed in Instance creation: Current timestamp added
        std::string id = "uninitialized";

        //Mutex s.t. only one thread has access to the file and the writer
        std::mutex log_mutex;

        Logging();
        uint64_t get_time();
        void check_id();

    public:
        static Logging& Instance();
        void set_id(std::string id);
        std::string get_filename();
        /**
         * Allows for a C-style use of the logger, like printf, using snprintf
         */
        template<class ...Args> void write(const char* f, Args&& ...args) {
            int size = snprintf(nullptr, 0, f, args...); //Determine the size of the resulting string without actually writing it
            std::string str(size, ' ');
            snprintf(& str[0], size + 1, f, args...);

            //Before flushing make sure that the Logger was initialized properly / that its ID was set
            check_id();
    
            //Get the current time, use this timestamp for logging purposes
            uint64_t time_now = get_time();

            //For the log file: csv, so escape '"'
            std::string log_string = std::string(str);
            std::string escaped_quote = std::string("\"\"");
            int pos = 0;
            while ((pos = log_string.find('"', pos)) != std::string::npos) {
                log_string.replace(pos, 1, escaped_quote);
                pos += escaped_quote.size();
            }
            //Also put the whole string in quotes
            log_string.insert(0, "\"");
            log_string += "\"";

            //Mutex for writing the message (file, writer) - is released when going out of scope
            std::lock_guard<std::mutex> lock(log_mutex);

            //Add the message to the log file
            file.open(filename, std::ios::app);
            file << id << "," << time_now << "," << log_string << std::endl;
            file.close();

            //Send the log message via RTI
            Log log(id, str, TimeStamp(time_now));
            logger.write(log);

            //Show the log message on the console
            std::cerr << "Log at time " << time_now << ": " << str << std::endl;
        }
};
