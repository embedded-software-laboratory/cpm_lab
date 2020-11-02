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

#include "LogLevelSetter.hpp"

LogLevelSetter::LogLevelSetter() :
    log_level_writer(
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<LogLevel>(cpm::ParticipantSingleton::Instance(), "logLevel"), 
        (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll() << dds::core::policy::Durability::TransientLocal())
    )
{
    
}

LogLevelSetter& LogLevelSetter::Instance()
{
    static thread_local std::unique_ptr<LogLevelSetter> _instance(new LogLevelSetter());

    return *_instance.get();
}

void LogLevelSetter::set_log_level(unsigned short log_level)
{
    //Create the LogLevel msg object
    LogLevel msg;
    msg.log_level(log_level);

    //Write the message / send it to all other participants within the domain
    log_level_writer.write(msg);

    //Priority 1 to always show this message in the logs - 
    //it is not a critical error per se, but a level change 
    //can be crucial for debugging and should thus always be indicated
    cpm::Logging::Instance().write(1, "Log level was changed to %i", static_cast<int>(log_level));
}