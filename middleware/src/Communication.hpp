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
 * \brief This class holds all readers and writers required for the middleware and provides access to them
 */
#include <string>
#include <sstream>
#include <functional>
#include <memory>
#include <vector>
#include <map>
#include <algorithm>

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
#include "VehicleStateList.hpp"

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/Timer.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/Writer.hpp"
#include "cpm/ReaderAbstract.hpp"
#include "cpm/Participant.hpp"

#include "VehicleCommandTrajectory.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandDirect.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"
#include "VehicleObservation.hpp"

#include "TypedCommunication.hpp"

using namespace std::placeholders;

/**
 * \brief This class is responsible for the communication between HLC, middleware and vehicle
 * These include message forwarding, collecting of vehicle states, timing messages and exchange of commands
 * For command exchange, we use the specialized class TypedCommunication, because we want to support it for different
 * command types (e.g. trajectories) which all have the same behaviour
 */
class Communication {
    private:
        //For HLC - communication
        cpm::Participant hlcParticipant;
        cpm::Writer<VehicleStateList> hlcStateWriter;
        cpm::ReaderAbstract<ReadyStatus> hlc_ready_status_reader;

        //Timing messages to HLC
        cpm::Writer<SystemTrigger> hlc_system_trigger_writer;
        cpm::AsyncReader<SystemTrigger> lcc_system_trigger_reader;

        //For Vehicle communication
        cpm::MultiVehicleReader<VehicleState> vehicleReader;

        //For vehicle observation
        cpm::MultiVehicleReader<VehicleObservation> vehicleObservationReader;

        //Communication for commands
        TypedCommunication<VehicleCommandTrajectory> trajectoryCommunication;
        TypedCommunication<VehicleCommandSpeedCurvature> speedCurvatureCommunication;
        TypedCommunication<VehicleCommandDirect> directCommunication;
    public:
        /**
         * \brief Constructor
         * \param hlcDomainNumber DDS domain number of the communication on the HLC (middleware and script)
         * \param vehicleStateListTopicName Topic name for vehicle state list messages
         * \param vehicleTrajectoryTopicName Topic name for trajectory messages
         * \param vehicleSpeedCurvatureTopicName Topic name for speed curvature messages
         * \param vehicleDirectTopicName Topic name for vehicle direct messages
         * \param _timer Required for current real or simulated timing information to check if answers of the HLC / script are received in time
         * \param vehicle_ids List of vehicle IDs for setup of the readers (ignore other data)
         */
        Communication(
            int hlcDomainNumber,
            std::string vehicleStateListTopicName,
            std::string vehicleTrajectoryTopicName,
            std::string vehicleSpeedCurvatureTopicName,
            std::string vehicleDirectTopicName,
            std::shared_ptr<cpm::Timer> _timer,
            std::vector<uint8_t> vehicle_ids
        ) 
        :hlcParticipant(hlcDomainNumber, "QOS_LOCAL_COMMUNICATION.xml")
        ,hlcStateWriter(hlcParticipant.get_participant(), vehicleStateListTopicName)
        ,hlc_ready_status_reader(hlcParticipant.get_participant(), "readyStatus", true, true, true)

        ,hlc_system_trigger_writer(hlcParticipant.get_participant(), "systemTrigger", true)
        ,lcc_system_trigger_reader(
            std::bind(&Communication::pass_through_system_trigger, this, _1),
            cpm::ParticipantSingleton::Instance(),
            cpm::get_topic<SystemTrigger>("systemTrigger"),
            true)

        ,vehicleReader(cpm::get_topic<VehicleState>("vehicleState"), vehicle_ids)

        ,vehicleObservationReader(cpm::get_topic<VehicleObservation>("vehicleObservation"), vehicle_ids)

        ,trajectoryCommunication(hlcParticipant.get_participant(), vehicleTrajectoryTopicName, _timer, vehicle_ids)
        ,speedCurvatureCommunication(hlcParticipant.get_participant(), vehicleSpeedCurvatureTopicName, _timer, vehicle_ids)
        ,directCommunication(hlcParticipant.get_participant(), vehicleDirectTopicName, _timer, vehicle_ids)
        {
        }

        /**
         * \brief This functions checks if messages of an HLC are within the given period / have been received at all
         * \param id ID of the HLC
         * \param t_now Current time
         * \param period_nanoseconds Periodicity of HLC messages (real time) or zero (simulated time)
         * \return False only for busy waiting of sim. time, if the ID could not be found / if no answer for the current time step was received
         */
        bool checkHLCResponseTime(uint8_t id, uint64_t t_now, uint64_t period_nanoseconds)
        {
            //Get latest response time of all message types
            auto latest_response_trajectory = trajectoryCommunication.getLatestHLCResponseTime(id);
            auto latest_response_curvature = speedCurvatureCommunication.getLatestHLCResponseTime(id);
            auto latest_response_direct = directCommunication.getLatestHLCResponseTime(id);

            //Check for irregularities
            // - No msg received
            if (! (latest_response_trajectory.has_value() || latest_response_curvature.has_value() || latest_response_direct.has_value()))
            {
                //Simulated time - we have not yet received any msg
                if (period_nanoseconds == 0)
                    return false;

                //Real time - just log the error, then return
                cpm::Logging::Instance().write(1, "HLC number %i has not yet sent any data", static_cast<int>(id));
                return true;
            }

            //  (Get highest of all values)
            auto max_latest_response = std::max(latest_response_trajectory.value_or(0), latest_response_curvature.value_or(0));
            max_latest_response = std::max(max_latest_response, latest_response_direct.value_or(0));

            // - Undesired behaviour - log this, but do not treat it as an error
            if (t_now < max_latest_response)
            {
                cpm::Logging::Instance().write(1, "Error: HLC %i answered with higher time than it was told to use", static_cast<int>(id));
            }

            // - Period missed (real time) / no current msg received (simulated time)
            uint64_t passed_time = (t_now - max_latest_response);
            if (passed_time > period_nanoseconds) {
                //Simulated time - we have not yet received the 'OK'/'finished' message for the current timestamp
                if (period_nanoseconds == 0)
                    return false;

                //Real-time
                std::stringstream stream;
                stream << "Timestep missed by HLC number " << static_cast<uint32_t>(id) << ", last response: " << max_latest_response 
                    << ", current time: " << t_now 
                    << ", periods missed: " << passed_time / period_nanoseconds;
                cpm::Logging::Instance().write(1, stream.str().c_str());
            }

            //Nothing else needs to be handled outside this function, so return true here
            return true;
        }

        //Only left for testing purposes, do not use for anything else
        std::unordered_map<uint8_t, uint64_t> getLastHLCResponseTimes() {
            //Go through the response times for all types, as different HLCs might use different types
            std::unordered_map<uint8_t, uint64_t> last_response_times_all_types;
            last_response_times_all_types = trajectoryCommunication.getLastHLCResponseTimes();

            const std::unordered_map<uint8_t, uint64_t> &curvature_times = speedCurvatureCommunication.getLastHLCResponseTimes();
            const std::unordered_map<uint8_t, uint64_t> &direct_times = directCommunication.getLastHLCResponseTimes();

            for (std::unordered_map<uint8_t, uint64_t>::const_iterator it = curvature_times.begin(); it != curvature_times.end(); ++it) {
                if (last_response_times_all_types.find(it->first) != last_response_times_all_types.end()) {
                    if (last_response_times_all_types[it->first] < curvature_times.at(it->first)) {
                        last_response_times_all_types[it->first] = it->second;
                    }
                }
                else {
                    last_response_times_all_types[it->first] = it->second;
                }
            }

            for (std::unordered_map<uint8_t, uint64_t>::const_iterator it = direct_times.begin(); it != direct_times.end(); ++it) {
                if (last_response_times_all_types.find(it->first) != last_response_times_all_types.end()) {
                    if (last_response_times_all_types[it->first] < direct_times.at(it->first)) {
                        last_response_times_all_types[it->first] = it->second;
                    }
                }
                else {
                    last_response_times_all_types[it->first] = it->second;
                }
            }

            return last_response_times_all_types;
        }

        void sendToHLC(VehicleStateList message) {
            hlcStateWriter.write(message);
        }

        /**
         * \brief Get most recent messages received by the vehicles (vehicle states) w.r.t. t_now
         */
        std::vector<VehicleState> getLatestVehicleMessages(uint64_t t_now) {
            VehicleState message;

            std::map<uint8_t, VehicleState> sample_out; 
            std::map<uint8_t, uint64_t> sample_age_out;

            vehicleReader.get_samples(t_now, sample_out, sample_age_out);

            std::vector<VehicleState> states;
            for (std::map<uint8_t, VehicleState>::iterator it = sample_out.begin(); it != sample_out.end(); ++it) {
                states.push_back(it->second);
            }

            return states;
        }

        /**
         * \brief Get most recent messages received by the IPS (vehicle observation) w.r.t. t_now
         */
        std::vector<VehicleObservation> getLatestVehicleObservationMessages(uint64_t t_now) {
            VehicleObservation message;

            std::map<uint8_t, VehicleObservation> sample_out; 
            std::map<uint8_t, uint64_t> sample_age_out;

            vehicleObservationReader.get_samples(t_now, sample_out, sample_age_out);

            std::vector<VehicleObservation> states;
            for (std::map<uint8_t, VehicleObservation>::iterator it = sample_out.begin(); it != sample_out.end(); ++it) {
                states.push_back(it->second);
            }

            return states;
        }

        /**
         * \brief Pass system trigger / timing messages from the LCC to the LCC
         */
        void pass_through_system_trigger(dds::sub::LoanedSamples<SystemTrigger>& samples) {
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    hlc_system_trigger_writer.write(sample.data());
                }
            }
        }

        /**
         * \brief The list of vehicles IDs passed to the Middleware shows how many different HLCs the Middleware is connected to.
         * Each of the HLCs needs to send an initial bootup message s.t. the Middleware knows that they are all online.
         */
        void wait_for_hlc_ready_msg(const std::vector<uint8_t>& vehicle_ids) {
            std::vector<std::string> vehicle_ids_string;
            for (uint8_t vehicle_id : vehicle_ids) {
                std::stringstream stream;
                stream << "hlc_" << static_cast<uint32_t>(vehicle_id);
                vehicle_ids_string.push_back(stream.str());
            }

            //Wait until all vehicle ids have been received at least once
            //Log if waiting for longer times
            unsigned int wait_cycles = 0;
            while(vehicle_ids_string.size() > 0) {
                for (auto data : hlc_ready_status_reader.take()) {
                    std::string source_id = data.source_id();
                    auto pos = std::find(vehicle_ids_string.begin(), vehicle_ids_string.end(), source_id);
                    if (pos != vehicle_ids_string.end()) {
                        vehicle_ids_string.erase(pos);
                    }
                }

                if (wait_cycles > 10)
                {
                    wait_cycles = 0;
                    std::stringstream remaining_ids;
                    for (auto id : vehicle_ids_string)
                    {
                        remaining_ids << id << " | ";
                    }

                    cpm::Logging::Instance().write(2, "Still waiting for ready messages from the HLC in the Middleware for IDs: %s", remaining_ids.str().c_str());
                }
                
                usleep(200000);
                ++wait_cycles;
            }
        }

        /**
         * \brief Update the current period start time stored in typed communication for internal checks
         * \param t_now Current period time, obtained by the cpm timer
         */
        void update_period_t_now(uint64_t t_now)
        {
            trajectoryCommunication.update_period_t_now(t_now);
            speedCurvatureCommunication.update_period_t_now(t_now);
            directCommunication.update_period_t_now(t_now);
        }
};