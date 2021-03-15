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

#include <iostream>
#include <algorithm>
#include <string>
#include <functional>
#include <vector>
#include <future>

#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Participant.hpp"

/**
 * \file AsyncReader.hpp
 */

namespace cpm 
{
    /**
     * \class AsyncReader
     * \brief This class is a wrapper for a data reader that uses an AsyncWaitSet to call a callback function whenever any new data is available
     * Template: Class of the message objects, depending on which IDL file is used
     * \ingroup cpmlib
     */ 
    template<class MessageType> 
    class AsyncReader
    {
    private:
        //Reader and waitset for receiving data and calling the callback function
        //! Internal DDS Subscriber instance
        dds::sub::Subscriber sub;
        //! Internal DDS Reader instance
        dds::sub::DataReader<MessageType> reader;
        //! Read condition to be able to async. receive data
        dds::core::cond::StatusCondition read_condition;
        //! Waitset as part of the read condition for async. data receiving
        rti::core::cond::AsyncWaitSet waitset;

        /**
         * \brief Returns qos for the settings s.t. the constructor becomes more readable
         * \param is_reliable If the QoS for DDS messages should be set to reliable (true) or best effort (false) messaging
         * \param is_transient_local If true, and if the Writer is still present, the Reader receives data that was sent before it went online
         */
        dds::sub::qos::DataReaderQos get_qos(bool is_reliable, bool is_transient_local)
        {
            //Initialize reader
            if (is_transient_local)
            {
                return (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll() << dds::core::policy::Durability::TransientLocal());
            }
            else if (is_reliable)
            {
                return (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll());
            }
            else
            {
                return (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::BestEffort() << dds::core::policy::History::KeepAll());
            }
        }

        /**
         * \brief Handler that takes unread samples, releases the waitset and calls the callback function provided by the user
         * \param func The callback function provided by the user
         */
        void handler(std::function<void(dds::sub::LoanedSamples<MessageType>&)> func);

        /**
         * \brief Handler that takes unread samples, copies them to a vector, releases the waitset and calls the callback function provided by the user
         * \param func The callback function provided by the user
         */
        void handler_vec(std::function<void(std::vector<MessageType>&)> func);
    public:
        /**
         * \brief Constructor for the AsynReader. This constructor is simpler and creates subscriber, topic etc on the cpm domain participant
         * The reader always uses History::KeepAll
         * \param func Callback function that is called by the reader if new data is available. Samples are passed to the function to be processed further.
         * \param topic_name The name of the topic that is supposed to be used by the reader
         * \param is_reliable If true, the used reader is set to be reliable, else best effort is expected
         * \param is_transient_local If true, the used reader is set to be transient local - in this case, it is also set to reliable and keep all
         */
        AsyncReader(
            std::function<void(std::vector<MessageType>&)> func, 
            std::string topic_name, 
            bool is_reliable = false,
            bool is_transient_local = false
        );

        /**
         * \brief Constructor for the AsynReader. This constructor is simpler and creates subscriber, topic etc on the cpm domain participant
         * The reader always uses History::KeepAll
         * \param func Callback function that is called by the reader if new data is available. Samples are passed to the function to be processed further.
         * \param participant Domain participant to specify in which domain the reader should operate
         * \param topic_name The name of the topic that is supposed to be used by the reader
         * \param is_reliable If true, the used reader is set to be reliable, else best effort is expected
         * \param is_transient_local If true, the used reader is set to be transient local - in this case, it is also set to reliable and keep all
         */
        AsyncReader(
            std::function<void(std::vector<MessageType>&)> func,
            cpm::Participant& participant, 
            std::string topic_name, 
            bool is_reliable = false,
            bool is_transient_local = false
        );

        /**
         * \brief Returns # of matched writers
         */
        size_t matched_publications_size();
    };

    template<class MessageType> 
    AsyncReader<MessageType>::AsyncReader(
        std::function<void(std::vector<MessageType>&)> func, 
        std::string topic_name, 
        bool is_reliable,
        bool is_transient_local
    )
    :sub(cpm::ParticipantSingleton::Instance())
    ,reader(sub, cpm::get_topic<MessageType>(topic_name), get_qos(is_reliable, is_transient_local))
    ,read_condition(reader)
    {
        //Call the callback function whenever any new data is available
        read_condition.enabled_statuses(dds::core::status::StatusMask::data_available()); 

        //Register the callback function
        read_condition->handler(std::bind(&AsyncReader::handler_vec, this, func));
        
        //Attach the read condition
        waitset.attach_condition(read_condition);
        
        //Start the waitset; from now on, whenever data is received the callback function is called
        waitset.start();
    }

    template<class MessageType> 
    AsyncReader<MessageType>::AsyncReader(
        std::function<void(std::vector<MessageType>&)> func, 
        cpm::Participant& participant,
        std::string topic_name, 
        bool is_reliable,
        bool is_transient_local
    )
    :sub(participant.get_participant())
    ,reader(sub, cpm::get_topic<MessageType>(participant.get_participant(), topic_name), get_qos(is_reliable, is_transient_local))
    ,read_condition(reader)
    {
        //Call the callback function whenever any new data is available
        read_condition.enabled_statuses(dds::core::status::StatusMask::data_available()); 

        //Register the callback function
        read_condition->handler(std::bind(&AsyncReader::handler_vec, this, func));
        
        //Attach the read condition
        waitset.attach_condition(read_condition);
        
        //Start the waitset; from now on, whenever data is received the callback function is called
        waitset.start();
    }

    template<class MessageType> 
    void AsyncReader<MessageType>::handler_vec(std::function<void(std::vector<MessageType>&)> func)
    {
        // Take all samples This will reset the StatusCondition
        dds::sub::LoanedSamples<MessageType> samples = reader.take();
        std::vector<MessageType> samples_vec;

        for (auto sample : samples)
        {
            if(sample.info().valid())
            {
                samples_vec.push_back(sample.data());
            }
        }

        // Release status condition in case other threads can process outstanding
        // samples
        waitset.unlock_condition(dds::core::cond::StatusCondition(reader));

        // Process sample 
        func(samples_vec);
    }

    template<class MessageType> 
    size_t AsyncReader<MessageType>::matched_publications_size()
    {
        auto matched_pub = dds::sub::matched_publications(reader);
        return matched_pub.size();
    }
}