#pragma once

#include <dds/topic/Topic.hpp>
#include <dds/topic/ddstopic.hpp>
#include <string>
#include "cpm/ParticipantSingleton.hpp"

namespace cpm
{
    template<typename T>
    dds::topic::Topic<T> get_topic(const dds::domain::DomainParticipant& participant, std::string topic_name)
    {
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