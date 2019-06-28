#include "cpm/ParticipantSingleton.hpp"
#include "InternalConfiguration.hpp"


namespace cpm 
{
    dds::domain::DomainParticipant& ParticipantSingleton::Instance() {

        static dds::core::QosProvider xml_file_provider(
            "no_multicast_qos.xml", 
            "no_multicast_library::no_multicast_profile"
        );

        static dds::domain::DomainParticipant myInstance(
            cpm::InternalConfiguration::Instance().get_dds_domain(),
            xml_file_provider.participant_qos("no_multicast_library::no_multicast_profile")
        );

        return myInstance;
    }
}