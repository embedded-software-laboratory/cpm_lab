#pragma once

#include <dds/topic/Topic.hpp>
#include <dds/topic/ddstopic.hpp>
#include <string>
#include <mutex>
#include "cpm/ParticipantSingleton.hpp"

/**
 * \brief Always use this class to create a DDS topic that is supposed to be used by more than one class: In RTI DDS, a topic can only be created once for each domain participant. Thus, if multiple classes need to use the same topic at some point, the topic object might need to be shared between them. These functions rely on topic creation and the dds::topic::find function. If a topic already exists, find is used, else a new topic is created. They allow to avoid unnecessary programming as the user does not need to manage the topic object in any way.
 */

namespace cpm
{
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


    template<typename T>
    dds::topic::Topic<T> get_topic(std::string topic_name)
    {
        return get_topic<T>(cpm::ParticipantSingleton::Instance(), topic_name);
    }
}