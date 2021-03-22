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
 * \test Tests VehicleIDFilteredTopic
 * 
 * - The filter for vehicle IDs
 * - First sends data for various IDs, then checks if they were received as expected
 * \ingroup cpmlib
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

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in VehicleIDFilteredTopic test" << std::endl << "\t";
    bool wait = true;
    while (wait)
    {
        usleep(10000); //Wait 10ms
        std::cout << "." << std::flush;

        auto matched_pub_1 = dds::sub::matched_publications(reader_vehicle42);
        auto matched_pub_2 = dds::sub::matched_publications(reader_vehicle11);

        if (writer_vehicleState.matched_subscriptions_size() > 0 && matched_pub_1.size() >=1 && matched_pub_2.size() >= 1)
            wait = false;
    }
    std::cout << std::endl;

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


    // wait for 'transmission' for up to 1 second
    std::vector<VehicleState> reader_samples11;
    std::vector<VehicleState> reader_samples42;
    for (int i = 0; i < 10; ++i)
    {
        auto reader_samples11_dds = reader_vehicle11.take();
        auto reader_samples42_dds = reader_vehicle42.take();

        for (auto& sample : reader_samples11_dds)
        {
            if (sample.info().valid()) reader_samples11.push_back(sample.data());
        }
        for (auto& sample : reader_samples42_dds)
        {
            if (sample.info().valid()) reader_samples42.push_back(sample.data());
        }

        //Abort early if condition is fulfilled, else wait and repeat read
        if (reader_samples11.size() >=1 && reader_samples42.size() >= 2) break;
        else usleep(100000);
    }

    // Look at the data sent before and check if they match the ID
    REQUIRE( reader_samples11.size() == 1 );
    REQUIRE( reader_samples42.size() == 2 );

    REQUIRE( reader_samples42[0].vehicle_id() == 42 );
    REQUIRE( reader_samples42[0].odometer_distance() == 2 );

    REQUIRE( reader_samples42[1].vehicle_id() == 42 );
    REQUIRE( reader_samples42[1].odometer_distance() == 6 );

    REQUIRE( reader_samples11[0].vehicle_id() == 11 );
    REQUIRE( reader_samples11[0].odometer_distance() == 3 );

}