#include "cpm/ParticipantSingleton.hpp"


namespace cpm 
{
    dds::domain::DomainParticipant& ParticipantSingleton::Instance() {
        static dds::domain::DomainParticipant myInstance(0);
        return myInstance;
    }
}