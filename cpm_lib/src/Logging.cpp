#include "cpm/Logging.hpp"

/**
 * \file Logging.cpp
 * \ingroup cpmlib
 */

namespace cpm {

    Logging::Logging() :
        logger("log", true)
    {
        //Get log level / logging verbosity
        log_level_reader = std::make_shared<cpm::AsyncReader<LogLevel>>(
            [this](std::vector<LogLevel>& samples){
                for(auto& data : samples)
                {
                    log_level.store(data.log_level());
                }
            },
            "logLevel",
            true,
            true
        );

        //Default log level value
        log_level.store(1);

        // Formatted start time for log filename
        char time_format_buffer[100];
        {
            struct tm* tm_info;
            time_t timer;
            time(&timer);
            tm_info = gmtime(&timer);
            strftime(time_format_buffer, 100, "%Y_%m_%d_%H_%M_%S", tm_info);
        }


        filename = "Log_";
        filename += time_format_buffer;
        filename += ".csv";

        file.open(filename, std::ofstream::out | std::ofstream::trunc);
        file << "ID,Level,Timestamp,Content" << std::endl;
        file.close();
    }

    Logging& Logging::Instance() {
        static Logging instance;
        return instance;
    }

    uint64_t Logging::get_time() {
        return cpm::get_time_ns();
    }

    void Logging::set_id(std::string _id) {
        //Mutex bc value could be set by different threads at once
        std::lock_guard<std::mutex> lock(log_mutex);

        id = _id;
    }

    std::string Logging::get_filename() {
        return filename;
    }

    void Logging::check_id() {
        //Mutex bc value could be set by different threads at once (id could be set with set_id while it is read)
        std::lock_guard<std::mutex> lock(log_mutex);

        if (id == "uninitialized") {
            fprintf(stderr, "Error: Logger ID was never set\n");
            fflush(stderr); 
            exit(EXIT_FAILURE);
        }
    }

}