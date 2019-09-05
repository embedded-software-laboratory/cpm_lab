#include "catch.hpp"
#include <memory>
#include <string>
#include <functional>
#include <random>
#include <atomic>
#include <algorithm>
#include <thread>
#include <vector>

#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/core/QosProvider.hpp>
#include <rti/core/cond/AsyncWaitSet.hpp>
#include <rti/core/ListenerBinder.hpp>

#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "VehicleState.hpp"
#include "../idl_compiled/VehicleStateList.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "Parameter.hpp"

#include "Communication.hpp"

/**
 * \brief Tests getLastHLCResponseTime and sendToHLC (middleware functions). Creates a fake HLC and a fake middleware to do so and skips one period on purpose. 
 */

TEST_CASE( "MiddlewareToHLCCommunication" ) {
    cpm::Logging::Instance().set_id("middleware_test");
    
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
    uint64_t period_nanoseconds = 50000000; //50ms
    uint64_t offset_nanoseconds = 1;
    bool simulated_time_allowed = true;
    bool simulated_time = false;

    //Initialize the timer
    std::shared_ptr<cpm::Timer> timer = cpm::Timer::create(node_id, period_nanoseconds, offset_nanoseconds, false, simulated_time_allowed, simulated_time);

    //Initialize the communication 
    std::shared_ptr<Communication> communication = std::make_shared<Communication>(hlcDomainNumber, hlcStateTopicName, vehicleStateTopicName, hlcTrajectoryTopicName, vehicleTrajectoryTopicName, hlcSpeedCurvatureTopicName, vehicleSpeedCurvatureTopicName, hlcDirectTopicName, vehicleDirectTopicName, vehicleID, timer, vehicle_ids);

    //HLC Writer
    dds::domain::DomainParticipant participant = dds::domain::find(hlcDomainNumber);
    dds::pub::DataWriter hlcWriter(dds::pub::Publisher(participant), dds::topic::find<dds::topic::Topic<VehicleCommandSpeedCurvature>>(participant, hlcSpeedCurvatureTopicName));

    //Data for checks
    std::vector<uint64_t> hlc_current_round_received;
    std::vector<bool> timestep_not_missed;
    
    //Round number: Used by the simulated middleware to count how often its callback was called & by the fake HLC to check if the current count matches the received count number
    std::vector<uint64_t> round_numbers;
    size_t hlc_reader_round_pos = 0;

    //Test which data was received by the HLC
	dds::sub::DataReader hlcReader(dds::sub::Subscriber(participant), dds::topic::find<dds::topic::Topic<VehicleStateList>>(participant, hlcStateTopicName));
    dds::core::cond::StatusCondition readCondition = dds::core::cond::StatusCondition(hlcReader);
    rti::core::cond::AsyncWaitSet waitset;
    readCondition.enabled_statuses(dds::core::status::StatusMask::data_available());
    readCondition->handler([&] {
        dds::sub::LoanedSamples<VehicleStateList> samples = hlcReader.take();
        waitset.unlock_condition(dds::core::cond::StatusCondition(hlcReader));

        // Check if received data matches sent data + order is kept (not guaranteed though)
        // Store the highest round value and require it to be as high as the current round value of the timer (see checks)
        uint64_t highest_round_value = 0;
        uint64_t current_round = 0;
        bool got_valid_sample = false;

        for (auto sample : samples) {
            if (sample.info().valid()) {
                got_valid_sample = true;
                //Store all received round values and the current round value for comparison
                if (sample.data().state_list().at(0).header().create_stamp().nanoseconds() >= highest_round_value) {
                    highest_round_value = sample.data().state_list().at(0).header().create_stamp().nanoseconds();
                }
            }
        }

        if (got_valid_sample) {
            current_round = round_numbers.at(hlc_reader_round_pos);
            ++hlc_reader_round_pos;
            hlc_current_round_received.push_back(highest_round_value);
        }

        //Miss a period on purpose
        if (current_round % 3 == 2) {
            rti::util::sleep(dds::core::Duration::from_millisecs(51));
        }

        VehicleCommandSpeedCurvature curv(vehicleID, Header(TimeStamp(1), TimeStamp(1)), 0, 0);
        hlcWriter.write(curv);
    });
    waitset.attach_condition(readCondition);
    waitset.start();

    //Wait for setup
    rti::util::sleep(dds::core::Duration::from_millisecs(5));

    //Run the fake middleware (use communication class as the middleware does)
    using namespace std::placeholders;
    timer->start_async([&](uint64_t t_now) {
        int roundNum;

        if (round_numbers.size() == 0) {
            roundNum = 0;
        }
        else {
            roundNum = round_numbers.at(round_numbers.size() - 1) + 1;
        }

        //Get the last response time of the HLC -> Do not send another start signal if it is still calculating a solution
        // -> Print an error message if a period has been missed
        std::map<uint8_t, uint64_t> lastHLCResponseTimes;
        lastHLCResponseTimes = communication->getLastHLCResponseTimes();
        uint64_t lastHLCResponseTime = 0;
        if (lastHLCResponseTimes.find(vehicleID) != lastHLCResponseTimes.end()) {
            lastHLCResponseTime = lastHLCResponseTimes[vehicleID];
        }
        bool period_missed = (t_now - lastHLCResponseTime) > period_nanoseconds;
        period_missed &= (lastHLCResponseTime != 0);

        if (!period_missed) {
            round_numbers.push_back(roundNum);

            VehicleState state;
            state.vehicle_id(vehicleID);
            state.header(Header(TimeStamp(roundNum), TimeStamp(roundNum)));
            std::vector<VehicleState> states;
            states.push_back(state);
            dds::core::vector<VehicleState> rti_state_list(states);
            VehicleStateList state_list;
            state_list.state_list(rti_state_list);

            communication->sendToHLC(state_list);
        }

        //Log error if a time step was missed when that should not happen, else require that the error is detected
        timestep_not_missed.push_back(((!(period_missed || lastHLCResponseTime > t_now)) || (roundNum % 3 == 0)));
    });

    rti::util::sleep(dds::core::Duration::from_millisecs(1000));
    timer->stop();
    rti::util::sleep(dds::core::Duration::from_millisecs(200));

    //Perform checks
    //The highest round value in each round (HLC) should match the current round value of the timer
    CHECK(round_numbers.size() == hlc_current_round_received.size());
    CHECK(std::equal(round_numbers.begin(), round_numbers.end(), hlc_current_round_received.begin()));

    //If a period was not missed on purpose, the timestep should not have been missed. Measuring whether a period was missed - if the last response of the HLC was received before the timer was called again - should be possible using getLastHLCResponseTime
    for (bool period_not_missed : timestep_not_missed) {
        CHECK(period_not_missed);
    }
}