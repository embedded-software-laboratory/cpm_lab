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

/**
 * \class Communication.hpp
 * \brief This class can be used to create all readers and writers required for the middleware whose type can change, e.g. if instead of the trajectory the speed + curvature are used as commands
 * WARNING: There is no error handling for incompatible types
 */
#include <optional>
#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <cassert>
#include <type_traits>

#include <dds/domain/DomainParticipant.hpp>
#include <dds/core/QosProvider.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>
#include <rti/core/ListenerBinder.hpp>
#include <dds/dds.hpp>
#include <dds/sub/DataReaderListener.hpp>
#include <dds/core/ddscore.hpp>

#include "VehicleState.hpp"
#include "VehicleCommandDirect.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleCommandSpeedCurvature.hpp"

#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/Timer.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"

using namespace std::placeholders;

template<class MessageType> class TypedCommunication {
    private:
        //For HLC - communication
        dds::topic::Topic<MessageType> hlcCommandTopic;
        cpm::AsyncReader<MessageType> hlcCommandReader;

        //For Vehicle communication
        dds::topic::Topic<MessageType> vehicleCommandTopic;
        dds::pub::DataWriter<MessageType> vehicleWriter;

        //Real time: Last receive time of HLC message (To check for violation of period) for each HLC ID
        //Simulated time: last response time should match the current time
        std::shared_ptr<cpm::Timer> timer;
        std::unordered_map<uint8_t, uint64_t> lastHLCResponseTimes;
        std::mutex map_mutex;

        //Handler for commands received by the HLC
        void handler(dds::sub::LoanedSamples<MessageType>& samples)
        {
            // Process sample 
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    uint64_t receive_timestamp = timer->get_time();

                    //First send the data to the vehicle
                    sendToVehicle(sample.data());

                    //Then check if the sent data was plausible -> TODO?
                    // - Check if the valid after time is correct - TODO: Make sure that header() exists?
                    //sample.data().header().valid_after_stamp().nanoseconds()

                    //Then update the last response time of the HLC that sent the data
                    std::lock_guard<std::mutex> lock(map_mutex);
                    lastHLCResponseTimes[sample.data().vehicle_id()] = receive_timestamp;
                }
            }
        }
    public:
        TypedCommunication(
            dds::domain::DomainParticipant& hlcParticipant,
            std::string vehicleCommandTopicName,
            std::shared_ptr<cpm::Timer> _timer)
        :hlcCommandTopic(hlcParticipant, vehicleCommandTopicName)
        ,hlcCommandReader(std::bind(&TypedCommunication::handler, this, _1), hlcParticipant, hlcCommandTopic)
        ,vehicleCommandTopic(cpm::ParticipantSingleton::Instance(), vehicleCommandTopicName)
        ,vehicleWriter(
            dds::pub::Publisher(cpm::ParticipantSingleton::Instance()),
            vehicleCommandTopic,
            (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::BestEffort()))
        ,timer(_timer)
        ,lastHLCResponseTimes()
        {
            static_assert(std::is_same<decltype(std::declval<MessageType>().vehicle_id()), uint8_t>::value, "IDL type must have a vehicle_id.");
        }

        /**
         * \brief Returns latest HLC response time or an empty optional if no entry could be found
         */
        std::optional<uint64_t> getLatestHLCResponseTime(uint8_t id) {
            std::lock_guard<std::mutex> lock(map_mutex);
            if (lastHLCResponseTimes.find(id) != lastHLCResponseTimes.end())
                return std::optional<uint64_t>(lastHLCResponseTimes[id]);
            
            return std::nullopt;
        }

        //Only left for testing purposes, do not use for anything else
        const std::unordered_map<uint8_t, uint64_t> getLastHLCResponseTimes() {
            std::lock_guard<std::mutex> lock(map_mutex);
            return lastHLCResponseTimes;
        }

        void sendToVehicle(MessageType message) {
            vehicleWriter.write(message);
        }
};