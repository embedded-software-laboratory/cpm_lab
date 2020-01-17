#pragma once

/**
 * \class Communication.hpp
 * \brief This class can be used to create all readers and writers required for the middleware whose type can change, e.g. if instead of the trajectory the speed + curvature are used as commands
 * WARNING: There is no error handling for incompatible types
 */
#include <string>
#include <functional>
#include <memory>
#include <map>
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
        std::map<uint8_t, uint64_t> lastHLCResponseTimes;

        //Handler for commands received by the HLC
        void handler(dds::sub::LoanedSamples<MessageType>& samples)
        {
            // Process sample 
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    //First send the data to the vehicle
                    sendToVehicle(sample.data());

                    //Then update the last response time of the HLC that sent the data
                    uint64_t receive_timestamp = timer->get_time();
                    lastHLCResponseTimes[sample.data().vehicle_id()] = receive_timestamp;
                }
            }
        }
    public:
        TypedCommunication(dds::domain::DomainParticipant& hlcParticipant, std::string hlcCommandTopicName, std::string vehicleCommandTopicName, std::shared_ptr<cpm::Timer> _timer) :
            hlcCommandTopic(hlcParticipant, hlcCommandTopicName),
            hlcCommandReader(std::bind(&TypedCommunication::handler, this, _1), hlcParticipant, hlcCommandTopic),

            vehicleCommandTopic(cpm::ParticipantSingleton::Instance(), vehicleCommandTopicName),
            vehicleWriter(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), vehicleCommandTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::BestEffort())),
            timer(_timer),
            lastHLCResponseTimes()
        {
            static_assert(std::is_same<decltype(std::declval<MessageType>().vehicle_id()), uint8_t>::value, "IDL type must have a vehicle_id.");
        }

        const std::map<uint8_t, uint64_t>& getLastHLCResponseTimes() {
            return lastHLCResponseTimes;
        }

        void sendToVehicle(MessageType message) {
            vehicleWriter.write(message);
        }
};