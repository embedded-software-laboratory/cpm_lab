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
#include <functional>
#include <random>
#include <atomic>
#include <algorithm>
#include <thread>
#include <vector>
#include <chrono>

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
#include "VehicleStateList.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "Parameter.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/ReaderAbstract.hpp"
#include "cpm/Writer.hpp"
#include "cpm/Participant.hpp"

#include "Communication.hpp"

/**
 * \test Tests communication from middleware to simulated HLC
 * 
 * Tests getLastHLCResponseTime and sendToHLC (middleware functions). 
 * Creates a fake HLC and a fake middleware to do so and skips one period on purpose. 
 * \ingroup middleware
 */
TEST_CASE( "MiddlewareToHLCCommunication" ) {
    cpm::Logging::Instance().set_id("middleware_test");
    
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
    uint64_t period_nanoseconds = 50000000; //50ms
    uint64_t offset_nanoseconds = 1;
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
        vehicle_ids);

    //HLC Writer
    dds::domain::DomainParticipant participant = dds::domain::find(hlcDomainNumber);
    auto cpm_participant = cpm::Participant(participant);
    cpm::Writer<VehicleCommandSpeedCurvature> hlcWriter(participant, vehicleSpeedCurvatureTopicName);

    //Data for checks
    std::vector<uint64_t> hlc_current_round_received;
    std::vector<bool> timestep_not_missed;
    bool t_now_correct = true;
    bool period_ms_correct = true;
    uint64_t t_now_value = 100;
    uint64_t period_ms_value = 200;
    
    //Round number: Used by the simulated middleware to count how often its callback was called & by the fake HLC to check if the current count matches the received count number
    std::vector<uint64_t> round_numbers;
    size_t hlc_reader_round_pos = 0;

    //Test which data was received by the HLC
	cpm::AsyncReader<VehicleStateList> hlcReader([&] (std::vector<VehicleStateList>& samples) {
        // Check if received data matches sent data + order is kept (not guaranteed though)
        // Store the highest round value and require it to be as high as the current round value of the timer (see checks)
        uint64_t highest_round_value = 0;
        uint64_t current_round = 0;
        bool got_valid_sample = false;

        for (auto& data : samples) {
            got_valid_sample = true;
            //Store all received round values and the current round value for comparison
            if (data.state_list().at(0).header().create_stamp().nanoseconds() >= highest_round_value) {
                highest_round_value = data.state_list().at(0).header().create_stamp().nanoseconds();
            }

            t_now_correct = data.t_now() == t_now_value;
            period_ms_correct = data.period_ms() == period_ms_value;
        }

        if (got_valid_sample) {
            current_round = round_numbers.at(hlc_reader_round_pos);
            ++hlc_reader_round_pos;
            hlc_current_round_received.push_back(highest_round_value);
        }

        //Miss a period on purpose
        if (current_round % 3 == 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(51));
        }

        VehicleCommandSpeedCurvature curv(vehicleID, Header(TimeStamp(1), TimeStamp(1)), 0, 0);
        hlcWriter.write(curv);
    },
    cpm_participant, vehicleStateListTopicName);

    //Wait for setup
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

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
        std::unordered_map<uint8_t, uint64_t> lastHLCResponseTimes;
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
            state_list.t_now(t_now_value);
            state_list.period_ms(period_ms_value);

            communication->sendToHLC(state_list);
        }

        //Log error if a time step was missed when that should not happen, else require that the error is detected
        timestep_not_missed.push_back(((!(period_missed || lastHLCResponseTime > t_now)) || (roundNum % 3 == 0)));
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    timer->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    //Perform checks
    //The highest round value in each round (HLC) should match the current round value of the timer
    CHECK(round_numbers.size() == hlc_current_round_received.size());
    CHECK(std::equal(round_numbers.begin(), round_numbers.end(), hlc_current_round_received.begin()));

    //If a period was not missed on purpose, the timestep should not have been missed. Measuring whether a period was missed - if the last response of the HLC was received before the timer was called again - should be possible using getLastHLCResponseTime
    for (bool period_not_missed : timestep_not_missed) {
        CHECK(period_not_missed);
    }

    //Check if t_now and period_ms work
    CHECK(t_now_correct);
    CHECK(period_ms_correct);
}