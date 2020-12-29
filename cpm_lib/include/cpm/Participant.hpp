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

#pragma once

#include <dds/core/QosProvider.hpp>
#include <dds/dds.hpp>
#include <dds/core/ddscore.hpp>

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

namespace cpm
{
    /**
     * \class Participant
     * \brief Creates a DDS Participant, use this for abstraction
     * Also allows for loading .xml QoS files
     * \ingroup cpmlib
     */
    class Participant
    {
        dds::domain::DomainParticipant dds_participant;

        Participant(const Participant&) = delete;
        Participant& operator=(const Participant&) = delete;
        Participant(const Participant&&) = delete;
        Participant& operator=(const Participant&&) = delete;

    public:
        /**
         * \brief Constructor for a participant 
         * \param domain_number Set the domain ID of the domain within which the communication takes place
         */
        Participant(int domain_number)
        :
            dds_participant(domain_number)
        { 
            
        }

        /**
         * \brief Constructor for a participant 
         * \param domain_number Set the domain ID of the domain within which the communication takes place
         * \param qos_file QoS settings to be imported from an .xml file
         */
        Participant(int domain_number, std::string qos_file)
        :
            dds_participant(domain_number, dds::core::QosProvider(qos_file).participant_qos())
        { 
            
        }

        /**
         * \brief Constructor for a participant 
         * \param participant A dds participant to be stored in this wrapper function (only for the middleware, replace after eProsima implementation)
         */
        Participant(dds::domain::DomainParticipant& participant)
        :
            dds_participant(participant)
        { 
            
        }
        
        dds::domain::DomainParticipant& get_participant()
        {
            return dds_participant;
        }
    };
}