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
}