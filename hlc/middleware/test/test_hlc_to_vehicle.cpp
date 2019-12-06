#include "catch.hpp"
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <random>
#include <algorithm>
#include <thread>

#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/core/QosProvider.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>
#include <rti/core/ListenerBinder.hpp>
#include <dds/core/vector.hpp>

#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "../idl_compiled/VehicleStateList.hpp"
#include "Parameter.hpp"

#include "Communication.hpp"

/**
 * Tests if data sent by a virtual HLC is received by a virtual vehicle, therefore if TypedCommunication receives and sends the data as required
 */

TEST_CASE( "HLCToVehicleCommunication" ) {
    cpm::Logging::Instance().set_id("middleware_test");

    //Data for the tests
    uint64_t max_rounds = 5;
    std::vector<uint64_t> received_round_numbers;

    {
        //Communication parameters
        int hlcDomainNumber = 1; 
        std::string hlcStateTopicName = "stateTopic"; 
        std::string vehicleStateTopicName = "vehicleState"; 
        std::string hlcTrajectoryTopicName = "trajectoryTopic"; 
        std::string vehicleTrajectoryTopicName = "vehicleCommandTrajectory"; 
        std::string hlcSpeedCurvatureTopicName = "speedCurvatureTopic"; 
        std::string vehicleSpeedCurvatureTopicName = "vehicleCommandSpeedCurvature"; 
        std::string hlcDirectTopicName = "directTopic"; 
        std::string vehicleDirectTopicName = "vehicleCommandDirect"; 
        int vehicleID = 0; 
        std::vector<uint8_t> vehicle_ids = { 0 };

        //Timer parameters
        std::string node_id = "middleware";
        uint64_t period_nanoseconds = 6000000; //6ms
        uint64_t offset_nanoseconds = 1;
        bool simulated_time_allowed = true;
        bool simulated_time = false;

        //Initialize the timer
        std::shared_ptr<cpm::Timer> timer = cpm::Timer::create(
            node_id,
            period_nanoseconds,
            offset_nanoseconds,
            false,
            simulated_time_allowed,
            simulated_time);

        //Initialize the communication 
        Communication communication(
            hlcDomainNumber,
            hlcStateTopicName,
            vehicleStateTopicName,
            hlcTrajectoryTopicName,
            vehicleTrajectoryTopicName,
            hlcSpeedCurvatureTopicName,
            vehicleSpeedCurvatureTopicName,
            hlcDirectTopicName,
            vehicleDirectTopicName,
            vehicleID,
            timer,
            vehicle_ids);

        //Thread that simulates the vehicle (only a reader is created). A waitset is attached to the reader and a callback function is created. In this function, round numbers are stored - the number of each round should be received.
        dds::sub::DataReader<VehicleCommandSpeedCurvature> vehicleReader(
            dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()),
            dds::topic::find<dds::topic::Topic<VehicleCommandSpeedCurvature>>(cpm::ParticipantSingleton::Instance(),
            vehicleSpeedCurvatureTopicName));
        dds::core::cond::StatusCondition readCondition(vehicleReader);
        rti::core::cond::AsyncWaitSet waitset;
        readCondition.enabled_statuses(dds::core::status::StatusMask::data_available());
        readCondition->handler([&] {
            dds::sub::LoanedSamples<VehicleCommandSpeedCurvature> samples = vehicleReader.take();
            waitset.unlock_condition(dds::core::cond::StatusCondition(vehicleReader));

            // Store received data for later checks
            for (auto sample : samples) {
                if (sample.info().valid()) {
                    received_round_numbers.push_back(sample.data().header().create_stamp().nanoseconds());
                }
            }
        });
        waitset.attach_condition(readCondition);
        waitset.start();

        //Sleep for some milliseconds just to make sure that the reader has been initialized properly
        rti::util::sleep(dds::core::Duration::from_millisecs(50));

        //Send test data from a virtual HLC - only the round number matters here, which is transmitted using the timestamp value
        dds::domain::DomainParticipant participant = dds::domain::find(hlcDomainNumber);    
        dds::topic::Topic<VehicleCommandSpeedCurvature> topic = dds::topic::find<dds::topic::Topic<VehicleCommandSpeedCurvature>>(participant, hlcSpeedCurvatureTopicName);
        dds::pub::Publisher publisher = dds::pub::Publisher(participant);
        dds::pub::DataWriter<VehicleCommandSpeedCurvature> hlcWriter(publisher, topic);
        for (uint64_t i = 0; i <= max_rounds; ++i) {
            //Send data and wait
            rti::util::sleep(dds::core::Duration::from_millisecs(50));
            VehicleCommandSpeedCurvature curv(vehicleID, Header(TimeStamp(i), TimeStamp(i)), 0, 0);
            hlcWriter.write(curv);
        }
    }

    //Perform checks (wait a while before doing so, to make sure that everything has been received)
    rti::util::sleep(dds::core::Duration::from_millisecs(500));
    for(uint64_t i = 0; i <= max_rounds; ++i) {
        CHECK((std::find(received_round_numbers.begin(), received_round_numbers.end(), i) != received_round_numbers.end()));
    }
}