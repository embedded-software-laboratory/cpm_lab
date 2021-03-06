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

#include "cpm/ReaderAbstract.hpp"
#include "cpm/Writer.hpp"

/**
 * \test Tests Writer
 * \ingroup cpmlib
 */
TEST_CASE( "Writer" ) {
    cpm::Logging::Instance().set_id("test_writer");

    cpm::ReaderAbstract<VehicleState> vehicle_state_reader("sadfhasdflkasdhf", true, true, true);

    // Test the writer, find out if sample gets received
    cpm::Writer<VehicleState> vehicle_state_writer("sadfhasdflkasdhf", true, true, true);

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in Writer test" << std::endl << "\t";
    bool wait = true;
    while (wait)
    {
        usleep(10000); //Wait 10ms
        std::cout << "." << std::flush;

        if (vehicle_state_writer.matched_subscriptions_size() > 0 && vehicle_state_reader.matched_publications_size() > 0)
            wait = false;
    }
    std::cout << std::endl;

    //Send sample
    VehicleState vehicleState;
    vehicleState.vehicle_id(99);
    vehicle_state_writer.write(vehicleState);

    //Wait for data for up to 1 second
    auto samples = vehicle_state_reader.take();
    for (int i = 0; i < 9; ++i)
    {
        if (samples.size() <= 0)
        {
            usleep(100000);
            samples = vehicle_state_reader.take();
        }
        else break;
    }

    //Check that sample content is correct
    REQUIRE( samples.size() == 1 );
    REQUIRE( samples.begin()->vehicle_id() == 99 );
}