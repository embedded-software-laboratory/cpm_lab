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
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/stamp_message.hpp"

#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"

#include <vector>
#include <map>

TEST_CASE( "MultiVehicleReader" ) {
    
    // sender
    cpm::Writer<VehicleState> writer("asldkjfhslakdj");

    // receiver
    std::vector<uint8_t> vehicle_ids{1, 3, 7};
    cpm::MultiVehicleReader<VehicleState> reader(cpm::get_topic<VehicleState>("asldkjfhslakdj"), vehicle_ids);


    const uint64_t second = 1000000000ull;
    const uint64_t millisecond = 1000000ull;
    const uint64_t t0 = 1500000000ull * second;
    const uint64_t expected_delay = 400 * millisecond;

    // send
    for (uint64_t t_now = t0; t_now <= t0 + 10*second; t_now += second)
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance( (t_now-t0)/second );
        vehicleState.vehicle_id(1);
        cpm::stamp_message(vehicleState, t_now, expected_delay);
        writer.write(vehicleState);

        vehicleState.odometer_distance( (t_now-t0)/second + 1 );
        vehicleState.vehicle_id(3);
        cpm::stamp_message(vehicleState, t_now, expected_delay);
        writer.write(vehicleState);

        vehicleState.odometer_distance( (t_now-t0)/second + 2 );
        vehicleState.vehicle_id(7);
        cpm::stamp_message(vehicleState, t_now, expected_delay);
        writer.write(vehicleState);

        //Should be ignored
        vehicleState.odometer_distance( (t_now-t0)/second + 2 );
        vehicleState.vehicle_id(2);
        cpm::stamp_message(vehicleState, t_now, expected_delay);
        writer.write(vehicleState);

        usleep(10000);
    }

    sleep(1);

    // example read
    std::map<uint8_t, VehicleState> samples;
    std::map<uint8_t, uint64_t> samples_age;
    const uint64_t t_now = t0 + 5 * second + 300 * millisecond;
    reader.get_samples(t_now, samples, samples_age);

    REQUIRE( samples_age[vehicle_ids.at(0)] == 900 * millisecond );
    REQUIRE( samples[vehicle_ids.at(0)].odometer_distance() == 4 );
    REQUIRE( samples[vehicle_ids.at(0)].vehicle_id() == 1 );

    REQUIRE( samples_age[vehicle_ids.at(1)] == 900 * millisecond );
    REQUIRE( samples[vehicle_ids.at(1)].odometer_distance() == 5 );
    REQUIRE( samples[vehicle_ids.at(1)].vehicle_id() == 3 );

    REQUIRE( samples_age[vehicle_ids.at(2)] == 900 * millisecond );
    REQUIRE( samples[vehicle_ids.at(2)].odometer_distance() == 6 );
    REQUIRE( samples[vehicle_ids.at(2)].vehicle_id() == 7 );

    bool hasVehicleTwo = false;
    for (auto entry : samples) {
        if (entry.second.vehicle_id() == 2) {
            hasVehicleTwo = true;
        }
    }

    REQUIRE(!hasVehicleTwo);
}