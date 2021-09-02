#include "LogLevelSetter.hpp"

/**
 * \file LogLevelSetter.cpp
 * \ingroup lcc
 */

LogLevelSetter::LogLevelSetter() :
    log_level_writer("logLevel", true, true, true)
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