#include "catch.hpp"
#include <memory>
#include <string>
#include <functional>
#include <random>
#include <thread>
#include <vector>
#include <chrono>

#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include "VehicleStateList.hpp"
#include "Parameter.hpp"

#include "Communication.hpp"

/**
 * \test Tests if vehicle messages are received
 * 
 * This tests getLatestVehicleMessage (to make sure that most/all messages sent by two vehicle are actually received) 
 * [with sensor period = (timer period / 2) two messages from each vehicle are received on average by the reader, and only the latest one is returned]
 * \ingroup middleware
 */
TEST_CASE( "VehicleCommunication_Read" ) {
    cpm::Logging::Instance().set_id("middleware_test");
    
    //Communication parameters
    int hlcDomainNumber = 1; 
    std::string vehicleStateListTopicName = "vehicleStateList"; 
    std::string vehicleTrajectoryTopicName = "vehicleCommandTrajectory"; 
    std::string vehiclePathTrackingTopicName = "vehicleCommandPathTracking"; 
    std::string vehicleSpeedCurvatureTopicName = "vehicleCommandSpeedCurvature"; 
    std::string vehicleDirectTopicName = "vehicleCommandDirect"; 
    std::vector<uint8_t> assigned_vehicle_ids = { 0, 1 };
    std::vector<uint8_t> active_vehicle_ids = { 0, 1, 3 };
    int testMessagesAmount = 18;

    //Timer parameters
    std::string node_id = "middleware";
    uint64_t period_nanoseconds = 6000000; //6ms
    uint64_t offset_nanoseconds = 1;
    uint64_t sensor_period = 3; //3ms
    bool simulated_time_allowed = true;
    bool simulated_time = false;

    //Initialize the timer
    std::shared_ptr<cpm::Timer> timer = cpm::Timer::create(node_id, period_nanoseconds, offset_nanoseconds, false, simulated_time_allowed, simulated_time);

    //Initialize the communication
    std::shared_ptr<Communication> communication = std::make_shared<Communication>(
        hlcDomainNumber,
        vehicleStateListTopicName,
        vehicleTrajectoryTopicName,
        vehiclePathTrackingTopicName,
        vehicleSpeedCurvatureTopicName,
        vehicleDirectTopicName,
        timer,
        assigned_vehicle_ids,
        active_vehicle_ids);

    //Test data
    std::vector<uint64_t> received_timestamps_vehicle_0;
    std::vector<uint64_t> received_timestamps_vehicle_1;
    std::vector<uint64_t> received_timestamps_vehicle_3;

    //Test which data was received by the middleware - store the timestamp in each round
    using namespace std::placeholders;
    timer->start_async([&](uint64_t t_now) {
        std::vector<VehicleState> state_list = communication->getLatestVehicleMessages(t_now);
        std::cout << state_list.size() << std::endl;
        received_timestamps_vehicle_0.push_back(state_list.at(0).header().create_stamp().nanoseconds());
        received_timestamps_vehicle_1.push_back(state_list.at(1).header().create_stamp().nanoseconds());
        received_timestamps_vehicle_3.push_back(state_list.at(2).header().create_stamp().nanoseconds());
    });

    //Send random data from three vehicle dummies to the Middleware
    cpm::Writer<VehicleState> vehicle_0_writer("vehicleState");
    cpm::Writer<VehicleState> vehicle_1_writer("vehicleState");
    cpm::Writer<VehicleState> vehicle_3_writer("vehicleState");

    for (int stamp_number = 0; stamp_number <= testMessagesAmount; ++stamp_number) {
        //Create random variable
        static thread_local std::mt19937 generator;
	    std::uniform_real_distribution<double> distribution(0.0, 3.0);
        double changingParam = distribution(generator);

		VehicleState first_vehicle_state;
        first_vehicle_state.vehicle_id(active_vehicle_ids.at(0));
        first_vehicle_state.header(Header(TimeStamp(stamp_number), TimeStamp(stamp_number)));
        first_vehicle_state.pose(Pose2D(0, 1 * changingParam, 2 * changingParam));
        first_vehicle_state.battery_voltage(3 * changingParam);

        VehicleState second_vehicle_state;
        second_vehicle_state.vehicle_id(active_vehicle_ids.at(1));
        second_vehicle_state.header(Header(TimeStamp(stamp_number), TimeStamp(stamp_number)));
        second_vehicle_state.pose(Pose2D(0, 1 * changingParam, 2 * changingParam));
        second_vehicle_state.battery_voltage(3 * changingParam);

        VehicleState third_vehicle_state;
        third_vehicle_state.vehicle_id(active_vehicle_ids.at(2));
        third_vehicle_state.header(Header(TimeStamp(stamp_number), TimeStamp(stamp_number)));
        third_vehicle_state.pose(Pose2D(0, 1 * changingParam, 2 * changingParam));
        third_vehicle_state.battery_voltage(3 * changingParam);

		vehicle_0_writer.write(first_vehicle_state);
        vehicle_1_writer.write(second_vehicle_state);
        vehicle_3_writer.write(third_vehicle_state);
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint64_t>(sensor_period)));
    }

    timer->stop();

    //Perform tests - check that no more than one stamp was missed
    for (size_t i = 1; i < received_timestamps_vehicle_0.size(); ++i) {
        CHECK(received_timestamps_vehicle_0.at(i) - received_timestamps_vehicle_0.at(i - 1) <= 2);
    }
    for (size_t i = 1; i < received_timestamps_vehicle_1.size(); ++i) {
        CHECK(received_timestamps_vehicle_1.at(i) - received_timestamps_vehicle_1.at(i - 1) <= 2);
    }
    for (size_t i = 1; i < received_timestamps_vehicle_3.size(); ++i) {
        CHECK(received_timestamps_vehicle_3.at(i) - received_timestamps_vehicle_3.at(i - 1) <= 2);
    }
    //Check that the last message (-> newest message in the last step) was received
    CHECK(received_timestamps_vehicle_0.at(received_timestamps_vehicle_0.size() - 1) == testMessagesAmount);
    CHECK(received_timestamps_vehicle_1.at(received_timestamps_vehicle_1.size() - 1) == testMessagesAmount);
    CHECK(received_timestamps_vehicle_3.at(received_timestamps_vehicle_3.size() - 1) == testMessagesAmount);
}
