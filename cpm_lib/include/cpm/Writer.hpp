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

#include <dds/pub/ddspub.hpp>
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

#include <dds/core/QosProvider.hpp>
#include <dds/dds.hpp>
#include <dds/core/ddscore.hpp>

// #include <experimental/filesystem>

namespace cpm
{
    /**
     * \class Writer
     * \brief Creates a DDS Writer that can be used for writing / publishing messages
     * This encapsulation allows for changes e.g. in the participant or QoS without 
     * the need to change the implementation across the whole project
     * \ingroup cpmlib
     */
    template<typename T>
    class Writer
    {
        dds::pub::DataWriter<T> dds_writer;

        Writer(const Writer&) = delete;
        Writer& operator=(const Writer&) = delete;
        Writer(const Writer&&) = delete;
        Writer& operator=(const Writer&&) = delete;

        /**
         * \brief Returns qos for the settings s.t. the constructor becomes more readable
         */
        dds::pub::qos::DataWriterQos get_qos(bool is_reliable, bool history_keep_all, bool is_transient_local)
        {
            auto qos = dds::pub::qos::DataWriterQos();

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
         * \brief Constructor for a writer which is communicating within the ParticipantSingleton
         * Allows to set the topic name and some QoS settings
         * \param topic Name of the topic to write in
         * \param reliable Set the writer to be reliable (true) or use best effort (false, default)
         * \param history_keep_all Keep all received messages (true) or not (false, default) (relevant for the reader)
         * \param transient_local Resent messages sent before a new participant joined to that participant (true) or not (false, default)
         */
        Writer(std::string topic, bool reliable = false, bool history_keep_all = false, bool transient_local = false)
        :dds_writer(dds::pub::Publisher(ParticipantSingleton::Instance()), cpm::get_topic<T>(topic), get_qos(reliable, history_keep_all, transient_local))
        { 
            
        }

        /**
         * \brief Constructor for a writer which is communicating within the ParticipantSingleton
         * Allows to set the topic name and some QoS settings
         * \param topic Name of the topic to write in
         * \param qos_xml_path Path for setting additional QoS
         * \param library The loaded library to use
         */
        Writer(std::string topic, std::string qos_xml_path, std::string library)
        :dds_writer(dds::pub::Publisher(ParticipantSingleton::Instance()), cpm::get_topic<T>(topic), dds::core::QosProvider(qos_xml_path, library).datawriter_qos())
        { 
        
        }

        /**
         * \brief Constructor for a writer that communicates within another domain
         * Allows to set the topic name and some QoS settings
         * \param _participant The domain (participant) in which to write
         * \param topic Name of the topic to write in
         * \param reliable Set the writer to be reliable (true) or use best effort (false, default)
         * \param history_keep_all Keep all received messages (true) or not (false, default) (relevant for the reader)
         * \param transient_local Resent messages sent before a new participant joined to that participant (true) or not (false, default)
         */
        Writer(
            dds::domain::DomainParticipant & _participant, 
            std::string topic, 
            bool reliable = false, 
            bool history_keep_all = false, 
            bool transient_local = false
        )
        :dds_writer(dds::pub::Publisher(_participant), cpm::get_topic<T>(_participant, topic), get_qos(reliable, history_keep_all, transient_local))
        { 
            
        }
        
        void write(T msg)
        {
            dds_writer.write(msg);
        }

        /**
         * \brief Returns # of matched writers, needs template parameter for topic type
         */
        size_t matched_subscriptions_size()
        {
            auto matched_sub = dds::pub::matched_subscriptions(dds_writer);
            return matched_sub.size();
        }
    };
}