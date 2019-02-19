#pragma once

/**
 * \class Logging.hpp
 * \brief This class can be used to log all relevant information or errors during runtime. These information are transmitted to the LabControlCenter.
 */

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <dds/domain/DomainParticipant.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/dds.hpp>
#include <dds/core/ddscore.hpp>

#include "ParameterRequest.hpp"

#include "cpm/ParticipantSingleton.hpp"

class Logging {
    Logging(Logging const&) = delete;
    Logging(Logging&&) = delete; 
    Logging& operator=(Logging const&) = delete;
    Logging& operator=(Logging &&) = delete;

    private:
        //DDS Writer for Logging
        dds::topic::Topic<ParameterRequest> loggingTopic;
        dds::pub::DataWriter<ParameterRequest> logger;

        //File for logging
        std::ofstream file;
        std::string filename = "Log.txt";

        Logging();

    public:
        static Logging& Instance();
        void log(std::string msg);
        template <typename T> Logging& operator<< (const T& log);
}