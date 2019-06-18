#include "cpm/ParticipantSingleton.hpp"
#include "InternalConfiguration.hpp"


namespace cpm 
{
    dds::domain::DomainParticipant& ParticipantSingleton::Instance() {
        static dds::domain::DomainParticipant myInstance(cpm::InternalConfiguration::Instance().get_dds_domain());
        return myInstance;
    }
}