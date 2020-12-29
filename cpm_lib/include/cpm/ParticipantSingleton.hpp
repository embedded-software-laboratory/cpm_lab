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

#include <dds/domain/DomainParticipant.hpp>

namespace cpm 
{
    /**
     * \class ParticipantSingleton
     * \brief Domain Participant Singleton for 
     * Domain 0 - this interface can be used to retrieve 
     * a Participant to create reader / writer / ... for 
     * this Domain
     * A domain participant is required to create reader 
     * and writer / publisher and subscriber. This interface 
     * can be used to retrieve the "default" participant that 
     * is used to communicate with e.g. the vehicles. Unless 
     * communication in another domain or QoS settings for the 
     * participant (instead of QoS settings for reader / writer / ...) 
     * are required, use this participant.
     * \ingroup cpmlib
     */
    class ParticipantSingleton
    {
        ParticipantSingleton() = delete;
        ParticipantSingleton(ParticipantSingleton const&) = delete;
        ParticipantSingleton(ParticipantSingleton&&) = delete; 
        ParticipantSingleton& operator=(ParticipantSingleton const&) = delete;
        ParticipantSingleton& operator=(ParticipantSingleton &&) = delete;

    public:
        /**
         * \brief Retrieve the participant singleton with this function
         * \return A participant
         */
        static dds::domain::DomainParticipant& Instance();

    };
}