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
            property.set(std::make_pair<std::string, std::string>
                ("dds.transport.UDPv4.builtin.multicast_enabled", "0"));

            std::vector<std::string> initial_peer_list { "builtin.udpv4://127.0.0.1" };

            const std::string initial_peer_cfg = cpm::InternalConfiguration::Instance().get_dds_initial_peer();
            if(!initial_peer_cfg.empty())
            {
                initial_peer_list.push_back(initial_peer_cfg);
            }

            auto &discovery = domainParticipantQos.policy<rti::core::policy::Discovery>();
            discovery.initial_peers(initial_peer_list);
            discovery.multicast_receive_addresses(std::vector<std::string>{});

            myInstance = std::make_shared<dds::domain::DomainParticipant>(
                cpm::InternalConfiguration::Instance().get_dds_domain(),
                domainParticipantQos
            );
        }


        return *myInstance;
    }
}