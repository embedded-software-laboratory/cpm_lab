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

#include <dds/topic/Topic.hpp>
#include <dds/topic/ddstopic.hpp>
#include <string>
#include <mutex>
#include "cpm/ParticipantSingleton.hpp"

namespace cpm
{
    /**
     * \brief Always use this function to create a DDS topic that 
     * is supposed to be used by more than one class: In RTI DDS, 
     * a topic can only be created once for each domain participant. 
     * Thus, if multiple classes need to use the same topic at some 
     * point, the topic object might need to be shared between 
     * them. These functions rely on topic creation and the 
     * dds::topic::find function. If a topic already exists, 
     * find is used, else a new topic is created. They allow to 
     * avoid unnecessary programming as the user does not need 
     * to manage the topic object in any way.
     * \param participant The cpm participant the topic is part of
     * \param topic_name The name of the topic
     * \ingroup cpmlib
     */
    template<typename T>
    dds::topic::Topic<T> get_topic(const dds::domain::DomainParticipant& participant, std::string topic_name)
    {
        static std::mutex get_topic_mutex;
        std::lock_guard<std::mutex> lock(get_topic_mutex);

        try
        {
            dds::topic::Topic<T> topic = dds::topic::find<dds::topic::Topic<T>>(participant, topic_name);
            if (topic == dds::core::null) {
                topic = dds::topic::Topic<T>(participant, topic_name);
            }

            return topic;
        }
        catch(...)
        {
            return dds::topic::Topic<T>(participant, topic_name);
        }
    }

    /**
     * Only for the ParticipantSingleton-Topics
     * 
     * Always use this function to create a DDS topic that 
     * is supposed to be used by more than one class: In RTI DDS, 
     * a topic can only be created once for each domain participant. 
     * Thus, if multiple classes need to use the same topic at some 
     * point, the topic object might need to be shared between 
     * them. These functions rely on topic creation and the 
     * dds::topic::find function. If a topic already exists, 
     * find is used, else a new topic is created. They allow to 
     * avoid unnecessary programming as the user does not need 
     * to manage the topic object in any way.
     * \param topic_name The name of the topic
     * \ingroup cpmlib
     */
    template<typename T>
    dds::topic::Topic<T> get_topic(std::string topic_name)
    {
        return get_topic<T>(cpm::ParticipantSingleton::Instance(), topic_name);
    }
}