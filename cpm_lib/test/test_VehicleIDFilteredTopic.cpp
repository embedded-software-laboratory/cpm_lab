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
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"

#include "cpm/Writer.hpp"

/**
 * Tests:
 * - The filter for vehicle IDs
 * - First sends data for various IDs, then checks if they were received as expected
 */

TEST_CASE( "VehicleIDFilteredTopic" ) {

    auto participant = cpm::ParticipantSingleton::Instance();
    dds::topic::Topic<VehicleState> topic_vehicle_state(participant, "my_topic_name");

    // One writer for all vehicle state test packages
    cpm::Writer<VehicleState> writer_vehicleState("my_topic_name");

    // Two filters for both vehicles; IDs: 42 and 11 (others are therefore ignored)
    cpm::VehicleIDFilteredTopic<VehicleState> topic_vehicle42_state(topic_vehicle_state, 42);
    cpm::VehicleIDFilteredTopic<VehicleState> topic_vehicle11_state(topic_vehicle_state, 11);

    // Reader for state packages with these IDs
    dds::sub::DataReader<VehicleState> reader_vehicle42(dds::sub::Subscriber(participant), topic_vehicle42_state, (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll()));
    dds::sub::DataReader<VehicleState> reader_vehicle11(dds::sub::Subscriber(participant), topic_vehicle11_state, (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll()));

    // allow time for DDS discovery
    sleep(1);

    // send state packages with different IDs, also those that should be ignored
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance(123);
        vehicleState.vehicle_id(3);
        writer_vehicleState.write(vehicleState);
    }

    {
        VehicleState vehicleState;    
        vehicleState.odometer_distance(2);
        vehicleState.vehicle_id(42);
        writer_vehicleState.write(vehicleState);
    }
    
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance(3);
        vehicleState.vehicle_id(11);
        writer_vehicleState.write(vehicleState);
    }
    
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance(6);
        vehicleState.vehicle_id(42);
        writer_vehicleState.write(vehicleState);
    }


    // wait for 'transmission'
    usleep(1000);


    // receive the data sent before and check if they match the ID
    auto reader_samples11 = reader_vehicle11.read();
    auto reader_samples42 = reader_vehicle42.read();

    REQUIRE( reader_samples11.length() == 1 );
    REQUIRE( reader_samples42.length() == 2 );

    REQUIRE( reader_samples42[0].data().vehicle_id() == 42 );
    REQUIRE( reader_samples42[0].data().odometer_distance() == 2 );

    REQUIRE( reader_samples42[1].data().vehicle_id() == 42 );
    REQUIRE( reader_samples42[1].data().odometer_distance() == 6 );

    REQUIRE( reader_samples11[0].data().vehicle_id() == 11 );
    REQUIRE( reader_samples11[0].data().odometer_distance() == 3 );

}