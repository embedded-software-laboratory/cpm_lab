#pragma once

#include <dds/topic/Topic.hpp>
#include <dds/topic/ddstopic.hpp>
#include <string>
#include <mutex>
#include "cpm/ParticipantSingleton.hpp"

namespace cpm
{
    template<typename T>
    dds::topic::Topic<T> get_topic(const dds::domain::DomainParticipant& participant, std::string topic_name)
    {
        static std::mutex get_topic_mutex;
        std::lock_guard<std::mutex> lock(get_topic_mutex);

        try
        {
            return dds::topic::Topic<T>(participant, topic_name);
        }
        catch(...)
        {
            return dds::topic::find<dds::topic::Topic<T>>(participant, topic_name);
        }
    }


    template<typename T>
    dds::topic::Topic<T> get_topic(std::string topic_name)
    {
        return get_topic<T>(cpm::ParticipantSingleton::Instance(), topic_name);
    }
}