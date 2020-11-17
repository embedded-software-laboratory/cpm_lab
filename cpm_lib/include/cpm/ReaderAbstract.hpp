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

/**
 * \class ReaderAbstract.hpp
 * \brief Creates a DDS Reader that provides the simple take() function for getting all samples received after the last call of "take()"
 * Abstraction from different DDS Reader implementations
 * Difference to cpm::Reader: That one is supposed to give the latest sample w.r.t. timing information in the header. ReaderAbstract works more general than that.
 */

#include <dds/sub/ddssub.hpp>
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

namespace cpm
{
    template<typename T>
    class ReaderAbstract
    {
        dds::sub::DataReader<T> dds_reader;

        ReaderAbstract(const ReaderAbstract&) = delete;
        ReaderAbstract& operator=(const ReaderAbstract&) = delete;
        ReaderAbstract(const ReaderAbstract&&) = delete;
        ReaderAbstract& operator=(const ReaderAbstract&&) = delete;

        /**
         * \brief Returns qos for the settings s.t. the constructor becomes more readable
         */
        dds::sub::qos::DataReaderQos get_qos(bool is_reliable, bool history_keep_all, bool is_transient_local)
        {
            auto qos = dds::sub::qos::DataReaderQos();

            if (is_reliable)
            {
                qos << dds::core::policy::Reliability::Reliable();
            }
            else
            {
                //Already implicitly given
                qos << dds::core::policy::Reliability::BestEffort();
            }

            if (history_keep_all)
            {
                qos << dds::core::policy::History::KeepAll();
            }

            if (is_transient_local)
            {
                qos << dds::core::policy::Durability::TransientLocal();
            }

            return qos;
        }

    public:
        /**
         * \brief Constructor for a ReaderAbstract which is communicating within the ParticipantSingleton
         * Allows to set the topic name and some QoS settings
         */
        ReaderAbstract(std::string topic, bool reliable = false, bool history_keep_all = false, bool transient_local = false)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), cpm::get_topic<T>(topic), get_qos(reliable, history_keep_all, transient_local))
        { 
            
        }

        /**
         * \brief Constructor for a ReaderAbstract that communicates within another domain
         * Allows to set the topic name and some QoS settings
         */
        ReaderAbstract(
            dds::domain::DomainParticipant & _participant, 
            std::string topic, 
            bool reliable = false, 
            bool history_keep_all = false, 
            bool transient_local = false
        )
        :dds_reader(dds::sub::Subscriber(_participant), cpm::get_topic<T>(_participant, topic), get_qos(reliable, history_keep_all, transient_local))
        { 
            
        }
        
        std::vector<T> take()
        {
            auto samples = dds_reader.take();
            std::vector<T> samples_vec;

            for (auto sample : samples)
            {
                if(sample.info().valid())
                {
                    samples_vec.push_back(sample.data());
                }
            }

            return samples_vec;
        }

        /**
         * \brief Returns # of matched writers, needs template parameter for topic type
         */
        template<typename MessageType>
        size_t matched_publications_size()
        {
            auto matched_pub = dds::sub::matched_publications<MessageType>(dds_reader);
            return matched_pub.size();
        }
    };
}