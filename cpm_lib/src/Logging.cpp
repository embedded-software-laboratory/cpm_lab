// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "cpm/Logging.hpp"

namespace cpm {

    Logging::Logging() :
        loggingTopic(cpm::get_topic<Log>(cpm::ParticipantSingleton::Instance(), "log")),
        logger(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), loggingTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()))
    {
        //Get log level / logging verbosity
        log_level_reader = std::make_shared<cpm::AsyncReader<LogLevel>>(
            [this](dds::sub::LoanedSamples<LogLevel>& samples){
                for(auto sample : samples)
                {
                    if(sample.info().valid())
                    {
                        log_level.store(sample.data().log_level());
                    }
                }
            },
            cpm::ParticipantSingleton::Instance(),
            cpm::get_topic<LogLevel>("logLevel"),
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