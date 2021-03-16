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
#include "cpm/ReaderAbstract.hpp"
#include "cpm/Writer.hpp"
#include "cpm/Logging.hpp"
#include "cpm/stamp_message.hpp"

/**
 * \test Tests ReaderAbstract
 * 
 * - The most basic cpm reader
 * \ingroup cpmlib
 */
TEST_CASE( "ReaderAbstract" ) {
    cpm::Logging::Instance().set_id("test_readerAbstract");

    // sender that sends various samples to the reader
    cpm::Writer<VehicleState> sample_writer("asldkjfhslakdj", true, true);

    // receiver - the cpm reader that receives the sample sent by the writer above
    cpm::ReaderAbstract<VehicleState> reader("asldkjfhslakdj", true, true);

    std::vector<double> expected_odometer_values;

    //Wait for reader setup
    usleep(10000);

    // send samples with different time stamps and data
    for (size_t i = 0; i < 5; ++i)
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance( static_cast<double>(i) );
        expected_odometer_values.push_back( static_cast<double>(i) );
        sample_writer.write(vehicleState);
        usleep(10000);
    }

    sleep(1);
    
    //Read should contain desired data
    auto samples = reader.take();
    std::vector<double> received_odometer_values;
    for (auto& sample : samples)
    {
        received_odometer_values.push_back(sample.odometer_distance());
    }

    for( auto expected_value : expected_odometer_values )
    {
        REQUIRE( std::find(received_odometer_values.begin(), received_odometer_values.end(), expected_value) != received_odometer_values.end() );
    }
}