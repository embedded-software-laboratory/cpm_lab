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

#include <dds/domain/DomainParticipant.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/dds.hpp>
#include <dds/core/ddscore.hpp>

#include "Log.hpp"

#include "cpm/ParticipantSingleton.hpp"

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

        Logging();
        uint64_t get_time();
        void check_id();

    public:
        static Logging& Instance();
        void set_id(std::string id);
        std::string get_filename();
        void flush(std::string &str);
        /**
         * Allows for a C-style use of the logger, like printf, using snprintf
         */
        template<class ...Args> void write(const char* f, Args&& ...args) {
            int size = snprintf(nullptr, 0, f, args...); //Determine the size of the resulting string without actually writing it
            std::string str(size, ' ');
            snprintf(& str[0], size + 1, f, args...);

            flush(str);
        }
};