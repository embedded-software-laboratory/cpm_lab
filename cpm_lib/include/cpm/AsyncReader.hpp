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
    /**
     * \class AsyncReader.hpp
     * \brief This class is a wrapper for a data reader that uses an AsyncWaitSet to call a callback function whenever any new data is available
     * Template: Class of the message objects, depending on which IDL file is used
     */ 

    template<class MessageType> 
    class AsyncReader
    {
    private:
        //Reader and waitset for receiving data and calling the callback function
        dds::sub::Subscriber sub;
        dds::sub::DataReader<MessageType> reader;
        dds::core::cond::StatusCondition read_condition;
        rti::core::cond::AsyncWaitSet waitset;
        std::string t_name;

        /*
         * \brief Handler that takes unread samples, releases the waitset and calls the callback function provided by the user
         * \param func The callback function provided by the user
         */
        void handler(std::function<void(dds::sub::LoanedSamples<MessageType>&)> func);
    public:
        /*
         * \brief Constructor for the AsynReader. Participant, topic and topic name need to be provided by the user, as well as the callback function for the reader.
         * \param topic_name The name of the topic that is supposed to be used by the reader
         * \param func Callback function that is called by the reader if new data is available. LoanedSamples are passed to the function to be processed further.
         * \param participant Domain participant to specify in which domain the reader should operate
         * \param topic The topic that is supposed to be used by the reader
         */
        AsyncReader(
            std::string topic_name, 
            std::function<void(dds::sub::LoanedSamples<MessageType>&)> func, 
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
        //Call the callback function whenever any new data is available
        read_condition.enabled_statuses(dds::core::status::StatusMask::data_available()); 

        //Register the callback function
        read_condition->handler(std::bind(&AsyncReader::handler, this, func));
        
        //Attach the read condition
        waitset.attach_condition(read_condition);
        
        //Start the waitset; from now on, whenever data is received the callback function is called
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