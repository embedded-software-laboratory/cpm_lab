#pragma once

#include <iostream>
#include <algorithm>
#include <string>
#include <functional>
#include <vector>
#include <future>

#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>

namespace cpm 
{


    template<class MessageType> 
    class AsyncReader
    {
    private:
        dds::sub::Subscriber sub;
        dds::sub::DataReader<MessageType> reader;
        dds::core::cond::StatusCondition read_condition;
        rti::core::cond::AsyncWaitSet waitset;
        std::string t_name;

        void handler(std::function<void(dds::sub::LoanedSamples<MessageType>&)> func);
    public:
        AsyncReader(
            std::string topic_name, 
            std::function<void(dds::sub::LoanedSamples<MessageType>&)>, 
            dds::domain::DomainParticipant & _participant, dds::topic::Topic<MessageType>& topic);
    };



    template<class MessageType> 
    AsyncReader<MessageType>::AsyncReader(
        std::string topic_name, 
        std::function<void(dds::sub::LoanedSamples<MessageType>&)> func, 
        dds::domain::DomainParticipant & _participant, dds::topic::Topic<MessageType>& topic
    )
    :sub(_participant)
    ,reader(sub, topic)
    ,read_condition(reader)
    {
        read_condition.enabled_statuses(dds::core::status::StatusMask::data_available());
        read_condition->handler(std::bind(&AsyncReader::handler, this, func));
        waitset.attach_condition(read_condition);
        waitset.start();

        t_name = topic_name;
    }




    template<class MessageType> 
    void AsyncReader<MessageType>::handler(
        std::function<void(dds::sub::LoanedSamples<MessageType>&)> func
    )
    {
        // Take all samples This will reset the StatusCondition
        dds::sub::LoanedSamples<MessageType> samples = reader.take();

        // Release status condition in case other threads can process outstanding
        // samples
        waitset.unlock_condition(dds::core::cond::StatusCondition(reader));

        // Process sample 
        func(samples);
    }
}