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

#pragma once

#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Writer.hpp"
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
    cpm::Writer<LogLevel> log_level_writer;

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