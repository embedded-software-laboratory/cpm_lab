#pragma once

/**
 * \class Communication.hpp
 * \brief This class can be used to create all readers and writers required for the middleware whose type can change, e.g. if instead of the trajectory the speed + curvature are used as commands
 */
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <cassert>

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

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/Timer.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"

template<class MessageType> class TypedCommunication {
    private:
        //For HLC - communication
        dds::topic::Topic<MessageType> hlcCommandTopic;
        dds::sub::Subscriber sub;
        dds::sub::DataReader<MessageType> hlcCommandReader;
        dds::core::cond::StatusCondition readCondition = dds::core::null;
        rti::core::cond::AsyncWaitSet waitset;

        //For Vehicle communication
        dds::topic::Topic<MessageType> vehicleCommandTopic;
        dds::pub::DataWriter<MessageType> vehicleWriter;

        //Real time: Last receive time of HLC message (To check for violation of period) for each HLC ID
        //Simulated time: last response time should match the current time
        std::shared_ptr<cpm::Timer> timer;
        std::map<uint8_t, uint64_t> lastHLCResponseTimes;

        //Handler for commands received by the HLC
        void handler()
        {
            // Take all samples - This will reset the StatusCondition
            dds::sub::LoanedSamples<MessageType> samples = hlcCommandReader.take();

            // Release status condition in case other threads can process outstanding
            // samples
            waitset.unlock_condition(dds::core::cond::StatusCondition(hlcCommandReader));

            // Process sample 
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    //First send the data to the vehicle
                    sendToVehicle(sample.data());

                    //Then update the last response time of the HLC that sent the data
                    lastHLCResponseTimes[sample.data().vehicle_id()] = timer->get_time();
                }
            }
        }
    public:
        TypedCommunication(dds::domain::DomainParticipant& hlcParticipant, std::string hlcCommandTopicName, std::string vehicleCommandTopicName, std::shared_ptr<cpm::Timer> _timer) :
            hlcCommandTopic(hlcParticipant, hlcCommandTopicName),
            sub(hlcParticipant),
            hlcCommandReader(sub, hlcCommandTopic),

            vehicleCommandTopic(cpm::ParticipantSingleton::Instance(), vehicleCommandTopicName),
            vehicleWriter(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), vehicleCommandTopic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::BestEffort())),
            timer(_timer),
            lastHLCResponseTimes()
        {
            static_assert(std::is_same<decltype(std::declval<MessageType>().vehicle_id()), uint8_t>::value, "IDL type must have a vehicle_id.");

            readCondition = dds::core::cond::StatusCondition(hlcCommandReader);
            readCondition.enabled_statuses(dds::core::status::StatusMask::data_available());
            readCondition->handler(std::bind(&TypedCommunication::handler, this));
            waitset.attach_condition(readCondition);
            waitset.start();
        }

        const std::map<uint8_t, uint64_t>& getLastHLCResponseTimes() {
            return lastHLCResponseTimes;
        }

        void sendToVehicle(MessageType message) {
            vehicleWriter.write(message);
        }
};