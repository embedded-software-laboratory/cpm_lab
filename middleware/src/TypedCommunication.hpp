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

#include "Header.hpp"
#include "VehicleState.hpp"
#include "VehicleCommandDirect.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleCommandSpeedCurvature.hpp"

#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/Timer.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/Writer.hpp"
#include "cpm/Participant.hpp"

using namespace std::placeholders;

/**
 * \brief This class is responsible for handling "any" message type (via templating) which can 
 * be received by the HLC script and must be forwarded to the vehicle
 * It also checks these messages for consistency and remembers when they were received (simulated or real time)
 * for further checks in main
 */
template<class MessageType> class TypedCommunication {
    private:
        //For HLC - communication
        cpm::AsyncReader<MessageType> hlcCommandReader;

        //For Vehicle communication
        cpm::Writer<MessageType> vehicleWriter;

        //Real time: Last receive time of HLC message (To check for violation of period) for each HLC ID
        //Simulated time: last response time should match the current time
        std::shared_ptr<cpm::Timer> timer;
        std::unordered_map<uint8_t, uint64_t> lastHLCResponseTimes;
        std::mutex map_mutex;

        //To check messages received from the HLC regarding their consistency with the vehicle IDs set for the middleware
        std::vector<uint8_t> vehicle_ids;

        //To check messages received from the HLC regarding their consistency with the timing managed by the middleware
        std::atomic<uint64_t> current_period_start{0}; //in ns

        //Handler for commands received by the HLC
        void handler(std::vector<MessageType>& samples)
        {
            // Process sample 
            for (auto& data : samples) {
                uint64_t receive_timestamp = timer->get_time();

                //First send the data to the vehicle
                sendToVehicle(data);

                //Then update the last response time of the HLC that sent the data
                std::lock_guard<std::mutex> lock(map_mutex);
                lastHLCResponseTimes[data.vehicle_id()] = receive_timestamp;

                //This might be problematic, but if we perform checks before sending the message then this 
                //might lead to a violation of timing boundaries

                //Then check if the sent data was plausible -> TODO? 
                // - Check if the valid after time is correct - TODO: Make sure that header() exists?
                //data.header().valid_after_stamp().nanoseconds()

                //1. Make sure that the set vehicle ID is valid (assertion of field done in constructor)
                auto set_id = data.vehicle_id();
                if(std::find(vehicle_ids.begin(), vehicle_ids.end(), set_id) == vehicle_ids.end())
                {
                    //ID should not have been sent, print warning
                    cpm::Logging::Instance().write(
                        1,
                        "Middleware received vehicle ID %i from HLC script - ID was not set to be used!",
                        static_cast<int>(set_id)
                    );
                }

                //2. Make sure that the creation timestamp is consistent with the current timing
                auto header_create_stamp = data.header().create_stamp().nanoseconds();
                auto current_time = cpm::get_time_ns();
                //  a) If the stamp is newer than the current time, then the timing function in the HLC script 
                //     must be wrong - after all, it runs on the same machine, so the same clock is being used
                if (header_create_stamp > current_time)
                {
                    cpm::Logging::Instance().write(
                        1,
                        "Middleware (ID %i) received creation stamp from HLC script that lies in the future - this must be a mistake",
                        static_cast<int>(set_id)
                    );
                }
                //  b) If the stamp is older than the beginning of the current period, than the HLC script took too long
                //     Missed periods are also checked in Communication, but only in terms of an absolute time diff
                if (header_create_stamp < current_period_start.load())
                {
                    cpm::Logging::Instance().write(
                        1,
                        "Middleware (ID %i): Received HLC message missed the current period",
                        static_cast<int>(set_id)
                    );
                }

                //Perform type specific checks (like amount of trajectory points for trajectory data)
                type_specific_msg_check(data);
            }
        }

        //Ignore warning that t_start is unused
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wunused-parameter"

        //Type specific / unspecific handlers
        void type_specific_msg_check(MessageType msg)
        {
            //Unspecific version, thus empty
            //Specializations can be found in the .cpp file
        }

        #pragma GCC diagnostic pop

        
    public:
        /**
         * \brief Constructor
         * \param hlcParticipant DDS participant instance for the communication with the HLC script
         * \param vehicleCommandTopicName Topic name for the selected message type
         * \param _timer To get the current time for real and simulated timing
         * \param _vehicle_ids List of IDs the Middleware and HLC are responsible for
         */
        TypedCommunication(
            cpm::Participant& hlcParticipant,
            std::string vehicleCommandTopicName,
            std::shared_ptr<cpm::Timer> _timer,
            std::vector<uint8_t> _vehicle_ids
        )
        :
        hlcCommandReader(std::bind(&TypedCommunication::handler, this, _1), hlcParticipant, vehicleCommandTopicName)
        ,vehicleWriter(vehicleCommandTopicName)
        ,timer(_timer)
        ,lastHLCResponseTimes()
        ,vehicle_ids(_vehicle_ids)
        {
            static_assert(std::is_same<decltype(std::declval<MessageType>().vehicle_id()), uint8_t>::value, "IDL type must have a vehicle_id.");
            static_assert(std::is_same<decltype(std::declval<MessageType>().header().create_stamp().nanoseconds()), unsigned long long>::value, "IDL type must use the Header IDL as header.");
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

        /**
         * \brief Update the current period start time stored in typed communication for internal checks
         * \param t_now Current period time, obtained by the cpm timer
         */
        void update_period_t_now(uint64_t t_now)
        {
            current_period_start.store(t_now);
        }
};