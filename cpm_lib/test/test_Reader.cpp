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
#include "cpm/Reader.hpp"
#include "cpm/Logging.hpp"
#include "cpm/stamp_message.hpp"

#include "cpm/Reader.hpp"
#include "cpm/Writer.hpp"

/**
 * \test Tests Reader
 * 
 * - The cpm reader
 * - If the reader returns the newest valid sample
 * \ingroup cpmlib
 */
TEST_CASE( "Reader" ) {
    cpm::Logging::Instance().set_id("test_reader");

    // sender that sends various samples to the reader
    cpm::Writer<VehicleState> sample_writer("asldkjfhslakdj");

    // receiver - the cpm reader that receives the sample sent by the writer above
    cpm::Reader<VehicleState> reader(cpm::get_topic<VehicleState>("asldkjfhslakdj"));

    const uint64_t second = 1000000000ull;
    const uint64_t millisecond = 1000000ull;
    const uint64_t t0 = 1500000000ull * second;
    const uint64_t expected_delay = 400 * millisecond;

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in Reader test" << std::endl << "\t";
    bool wait = true;
    while (wait)
    {
        usleep(10000); //Wait 10ms
        std::cout << "." << std::flush;

        if (reader.matched_publications_size() > 0 && sample_writer.matched_subscriptions_size() > 0)
            wait = false;
    }
    std::cout << std::endl;

    // send samples with different time stamps and data
    for (uint64_t t_now = t0; t_now <= t0 + 10*second; t_now += second)
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance( (t_now-t0)/second );
        vehicleState.vehicle_id(2);
        cpm::stamp_message(vehicleState, t_now, expected_delay);
        sample_writer.write(vehicleState);
        usleep(10000);
    }

    // example read, should contain the newest valid data depending on the value on t_now and the data sent before
    // Wait up to 1 second before "giving up"
    VehicleState sample;
    uint64_t sample_age;
    for (int i = 0; i < 10; ++i)
    {
        const uint64_t t_now = t0 + 5 * second + 300 * millisecond;
        reader.get_sample(t_now, sample, sample_age);

        if (sample.odometer_distance() == 4) break;
        else usleep(100000);
    }

    REQUIRE( sample_age == 900 * millisecond );
    REQUIRE( sample.odometer_distance() == 4 );
}