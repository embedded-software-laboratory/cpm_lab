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
#include "../idl_compiled/VehicleStateList.hpp"

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/Timer.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandDirect.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"
#include "VehicleObservation.hpp"

#include "TypedCommunication.hpp"

using namespace std::placeholders;
class Communication {
    private:
        //For HLC - communication
        dds::core::QosProvider local_comm_qos_provider;
        dds::domain::DomainParticipant hlcParticipant;
        dds::topic::Topic<VehicleStateList> hlcStateTopic;
        dds::pub::Publisher pub;
        dds::pub::DataWriter<VehicleStateList> hlcStateWriter;
        dds::topic::Topic<ReadyStatus> ready_topic;
        dds::sub::DataReader<ReadyStatus> hlc_ready_status_reader;

        //Timing messages to HLC
        dds::topic::Topic<SystemTrigger> trigger_topic;
        dds::pub::DataWriter<SystemTrigger> hlc_system_trigger_writer;
        cpm::AsyncReader<SystemTrigger> lcc_system_trigger_reader;

        //For Vehicle communication
        dds::topic::Topic<VehicleState> vehicleStateTopic;
        cpm::MultiVehicleReader<VehicleState> vehicleReader;

        //For vehicle observation
        dds::topic::Topic<VehicleObservation> vehicleObservationTopic;
        cpm::MultiVehicleReader<VehicleObservation> vehicleObservationReader;

        //Communication for commands
        TypedCommunication<VehicleCommandTrajectory> trajectoryCommunication;
        TypedCommunication<VehicleCommandSpeedCurvature> speedCurvatureCommunication;
        TypedCommunication<VehicleCommandDirect> directCommunication;

        //Misc
        int vehicleID;
    public:
        Communication(int hlcDomainNumber, std::string hlcStateTopicName, std::string vehicleStateTopicName, std::string hlcTrajectoryTopicName, std::string vehicleTrajectoryTopicName, std::string hlcSpeedCurvatureTopicName, std::string vehicleSpeedCurvatureTopicName, std::string hlcDirectTopicName, std::string vehicleDirectTopicName, int _vehicleID, std::shared_ptr<cpm::Timer> _timer, std::vector<uint8_t> vehicle_ids) :
            local_comm_qos_provider("QOS_LOCAL_COMMUNICATION.xml"),
            hlcParticipant(hlcDomainNumber, local_comm_qos_provider.participant_qos()),
            hlcStateTopic(hlcParticipant, hlcStateTopicName),
            pub(hlcParticipant),
            hlcStateWriter(pub, hlcStateTopic),
            ready_topic(hlcParticipant, "ready_hlc"),
            hlc_ready_status_reader(dds::sub::Subscriber(hlcParticipant), ready_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll())),
            trigger_topic(hlcParticipant, "system_trigger_hlc"),
            hlc_system_trigger_writer(dds::pub::Publisher(hlcParticipant), trigger_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable())),
            lcc_system_trigger_reader(std::bind(&Communication::pass_through_system_trigger, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<SystemTrigger>(cpm::ParticipantSingleton::Instance(), "system_trigger"), true),


            vehicleStateTopic(cpm::ParticipantSingleton::Instance(), vehicleStateTopicName),
            vehicleReader(vehicleStateTopic, vehicle_ids),

            vehicleObservationTopic(cpm::ParticipantSingleton::Instance(), "vehicleObservation"),
            vehicleObservationReader(vehicleObservationTopic, vehicle_ids),

            trajectoryCommunication(hlcParticipant, hlcTrajectoryTopicName, vehicleTrajectoryTopicName, _timer),
            speedCurvatureCommunication(hlcParticipant, hlcSpeedCurvatureTopicName, vehicleSpeedCurvatureTopicName, _timer),
            directCommunication(hlcParticipant, hlcDirectTopicName, vehicleDirectTopicName, _timer),
            vehicleID(_vehicleID)
        {
        }

        std::map<uint8_t, uint64_t> getLastHLCResponseTimes() {
            //Go through the response times for all types, as different HLCs might use different types
            std::map<uint8_t, uint64_t> last_response_times_all_types;
            last_response_times_all_types = trajectoryCommunication.getLastHLCResponseTimes();

            const std::map<uint8_t, uint64_t> &curvature_times = speedCurvatureCommunication.getLastHLCResponseTimes();
            const std::map<uint8_t, uint64_t> &direct_times = directCommunication.getLastHLCResponseTimes();

            for (std::map<uint8_t, uint64_t>::const_iterator it = curvature_times.begin(); it != curvature_times.end(); ++it) {
                if (last_response_times_all_types.find(it->first) != last_response_times_all_types.end()) {
                    if (last_response_times_all_types[it->first] < curvature_times.at(it->first)) {
                        last_response_times_all_types[it->first] = it->second;
                    }
                }
                else {
                    last_response_times_all_types[it->first] = it->second;
                }
            }

            for (std::map<uint8_t, uint64_t>::const_iterator it = direct_times.begin(); it != direct_times.end(); ++it) {
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
            while(vehicle_ids_string.size() > 0) {
                for (auto sample : hlc_ready_status_reader.take()) {
                    if (sample.info().valid()) {
                        std::string source_id = sample.data().source_id();
                        auto pos = std::find(vehicle_ids_string.begin(), vehicle_ids_string.end(), source_id);
                        if (pos != vehicle_ids_string.end()) {
                            vehicle_ids_string.erase(pos);
                        }
                    }
                }
                
                usleep(200000);
            }
        }
};