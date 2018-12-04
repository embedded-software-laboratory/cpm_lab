#pragma once
#include <dds/domain/DomainParticipant.hpp>

namespace cpm 
{
    class ParticipantSingleton
    {
        ParticipantSingleton() = delete;
        ParticipantSingleton(ParticipantSingleton const&) = delete;
        ParticipantSingleton(ParticipantSingleton&&) = delete; 
        ParticipantSingleton& operator=(ParticipantSingleton const&) = delete;
        ParticipantSingleton& operator=(ParticipantSingleton &&) = delete;

    public:
        static dds::domain::DomainParticipant& Instance();

    };
}