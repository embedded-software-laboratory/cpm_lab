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

#include "catch.hpp"
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <random>
#include <algorithm>
#include <thread>
#include <chrono>

#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "VehicleState.hpp"
#include "VehicleStateList.hpp"
#include "Parameter.hpp"

#include "Communication.hpp"

/**
 * Tests if data sent by a virtual vehicle is received by a fake middleware, therefore if Communication receives the vehicle data - tests getLatestVehicleMessage
 */

TEST_CASE( "VehicleToMiddlewareCommunication" ) {
    cpm::Logging::Instance().set_id("middleware_test");
    
    //Data for the tests
    uint64_t max_rounds = 5;
    std::vector<uint64_t> sent_round_numbers;
    std::vector<uint64_t> received_round_numbers;

    {
        //Communication parameters
        int hlcDomainNumber = 1; 
        std::string vehicleStateListTopicName = "vehicleStateList"; 
        std::string vehicleTrajectoryTopicName = "vehicleCommandTrajectory"; 
        std::string vehiclePathTrackingTopicName = "vehicleCommandPathTracking"; 
        std::string vehicleSpeedCurvatureTopicName = "vehicleCommandSpeedCurvature"; 
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
        std::shared_ptr<cpm::Timer> timer = cpm::Timer::create(node_id, period_nanoseconds, offset_nanoseconds, false, simulated_time_allowed, simulated_time);

        //Initialize the communication 
        Communication communication(
            hlcDomainNumber,
            vehicleStateListTopicName,
            vehicleTrajectoryTopicName,
            vehiclePathTrackingTopicName,
            vehicleSpeedCurvatureTopicName,
            vehicleDirectTopicName,
            timer,
            vehicle_ids
        );

        //Sleep for some milliseconds just to make sure that the readers have been initialized properly
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //Send test data from a virtual vehicle - only the round number matters here, which is transmitted using the timestamp value
        cpm::Writer<VehicleState> vehicleWriter("vehicleState");
        for (uint64_t i = 0; i <= max_rounds; ++i) {
            //Send data (first older data, then newer data - only the newer data should be returned by getLatestVehicleMessage) and wait
            VehicleState state_old;
            state_old.vehicle_id(vehicleID);
            state_old.header(Header(TimeStamp(3*i), TimeStamp(3*i)));
            vehicleWriter.write(state_old);

            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            VehicleState state_new;
            state_new.vehicle_id(vehicleID);
            state_new.header(Header(TimeStamp(3*i + 1), TimeStamp(3*i + 1)));
            vehicleWriter.write(state_new);

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            //Store the sent and received data for later comparison
            sent_round_numbers.push_back(3*i + 1);
            received_round_numbers.push_back(communication.getLatestVehicleMessages(3*i + 1).at(0).header().create_stamp().nanoseconds());
        }
    }

    // std::cout << "Sent numbers: ";
    // for (uint64_t sent_number : sent_round_numbers) {
    //     std::cout << sent_number << " | ";
    // }
    // std::cout << std::endl;

    // std::cout << "Received numbers: ";
    // for (uint64_t received_number : received_round_numbers) {
    //     std::cout << received_number << " | ";
    // }
    // std::cout << std::endl;

    //Perform checks
    for(uint64_t i = 0; i <= max_rounds; ++i) {
        CHECK(sent_round_numbers.at(i) == received_round_numbers.at(i));
    }
}