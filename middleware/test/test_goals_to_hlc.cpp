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
#include "CommonroadDDSGoalState.hpp"
#include "ReadyStatus.hpp"

#include "Communication.hpp"

/**
 * Tests if data sent by a virtual vehicle is received by a fake middleware, therefore if Communication receives the vehicle data - tests getLatestVehicleMessage
 */

TEST_CASE( "GoalToHLCCommunication" ) {
    cpm::Logging::Instance().set_id("middleware_test_log");
    
    //Data for the tests
    CommonroadDDSGoalState state_1;
    CommonroadDDSGoalState state_2;
    state_1.planning_problem_id(1);
    state_2.planning_problem_id(2);

    //Communication parameters - mostly required just to set up the test
    int hlcDomainNumber = 1; 
    std::string goalStateTopicName = "commonroad_dds_goal_states";
    std::string vehicleStateListTopicName = "vehicleStateList"; 
    std::string vehicleTrajectoryTopicName = "vehicleCommandTrajectory"; 
    std::string vehicleSpeedCurvatureTopicName = "vehicleCommandSpeedCurvature"; 
    std::string vehicleDirectTopicName = "vehicleCommandDirect"; 
    int vehicleID = 0; 
    std::vector<uint8_t> vehicle_ids = { 0 };

    //Timer parameters
    std::string node_id = "middleware_test";
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
        vehicleSpeedCurvatureTopicName,
        vehicleDirectTopicName,
        timer,
        vehicle_ids
    );

    //Initialize readers and writers to simulate HLC (ready messages) and LCC (goal state writer)
    dds::domain::DomainParticipant hlcParticipant = dds::domain::find(hlcDomainNumber);
    cpm::Writer<ReadyStatus> hlc_ready_status_writer(hlcParticipant, "readyStatus", true, true, true);
    cpm::Writer<CommonroadDDSGoalState> vehicleWriter(goalStateTopicName, true, true, true);
    cpm::ReaderAbstract<CommonroadDDSGoalState> hlc_goal_state_reader(hlcParticipant, goalStateTopicName, true, true, false);

    //Sleep for some milliseconds just to make sure that the readers and writers have been initialized properly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //Send test data from a virtual LCC
    vehicleWriter.write(state_1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vehicleWriter.write(state_2);

    //Send ready message to the middleware from a virtual HLC
    ReadyStatus hlc_ready;
    hlc_ready.source_id("hlc_0");
    hlc_ready_status_writer.write(hlc_ready);

    //Wait for the HLC ready msgs as the middleware does; GoalStates received by the middleware before the ready messages are now sent to the HLC
    communication.wait_for_hlc_ready_msg({0});

    //Now the HLC should have received these messages; wait a bit to make sure this happened
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<int32_t> received_ids;
    for (auto& sample : hlc_goal_state_reader.take())
    {
        received_ids.push_back(sample.planning_problem_id());
    }

    //Perform checks
    CHECK(received_ids.size() == 2);
    CHECK(std::find(received_ids.begin(), received_ids.end(), 1) != received_ids.end());
    CHECK(std::find(received_ids.begin(), received_ids.end(), 2) != received_ids.end());
}