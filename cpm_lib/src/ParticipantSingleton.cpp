#include "cpm/ParticipantSingleton.hpp"
#include "InternalConfiguration.hpp"
#include <dds/core/QosProvider.hpp>
#include <dds/domain/qos/DomainParticipantQos.hpp>
#include <memory>


namespace cpm 
{
    dds::domain::DomainParticipant& ParticipantSingleton::Instance() {

        static std::shared_ptr<dds::domain::DomainParticipant> myInstance = nullptr;

        if(!myInstance)
        {
            dds::domain::qos::DomainParticipantQos domainParticipantQos;

            auto &property = domainParticipantQos.policy<rti::core::policy::Property>();
            property.set(std::make_pair<std::string, std::string>("dds.transport.UDPv4.builtin.multicast_enabled", "0"));

            auto &discovery = domainParticipantQos.policy<rti::core::policy::Discovery>();

            discovery.initial_peers(std::vector<std::string>{
                "builtin.udpv4://127.0.0.1", 
                "192.168.1.101", 
                "192.168.1.102", 
                "192.168.1.103", 
                "192.168.1.104", 
                "192.168.1.105", 
                "192.168.1.106", 
                "192.168.1.107", 
                "192.168.1.108", 
                "192.168.1.109", 
                "192.168.1.110", 
                "192.168.1.111", 
                "192.168.1.112", 
                "192.168.1.113", 
                "192.168.1.114", 
                "192.168.1.115", 
                "192.168.1.116", 
                "192.168.1.117", 
                "192.168.1.118", 
                "192.168.1.119", 
                "192.168.1.249"
            });

            discovery.multicast_receive_addresses(std::vector<std::string>{});

            myInstance = std::make_shared<dds::domain::DomainParticipant>(
                cpm::InternalConfiguration::Instance().get_dds_domain(),
                domainParticipantQos
            );
        }


        return *myInstance;
    }
}