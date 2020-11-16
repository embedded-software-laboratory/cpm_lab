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
