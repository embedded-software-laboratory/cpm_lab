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
#include "cpm/Logging.hpp"
#include "cpm/get_topic.hpp"

#include "HLCHello.hpp"

#include <mutex>

#include "cpm/AsyncReader.hpp"
#include "cpm/Writer.hpp"

/**
 * \test Tests AsyncReader
 * WARNING: No other participant should be running while this test is running, or it will fail 
 * (due to potential answers to RTT requests by other participants in the network)
 * \ingroup cpmlib
 */
TEST_CASE( "AsyncReader" ) {
    cpm::Logging::Instance().set_id("test_async");

    //Create a reliable async reader to test
    std::vector<std::string> received_ids;
    std::mutex receive_mutex;
    cpm::AsyncReader<HLCHello> async_reader([&](std::vector<HLCHello>& samples){
        std::lock_guard<std::mutex> lock(receive_mutex);
        for(auto& data: samples)
        {
            received_ids.push_back(data.source_id());
        }
    },
    "async_reader_test", true, true);

    //Create a reliable writer to write test msgs to the reader
    cpm::Writer<HLCHello> test_writer("async_reader_test", true, true, true);

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in AsyncReader test" << std::endl << "\t";
    bool wait = true;
    while (wait)
    {
        usleep(100000); //Wait 100ms
        std::cout << "." << std::flush;

        if (test_writer.matched_subscriptions_size() > 0 && async_reader.matched_publications_size() > 0)
            wait = false;
    }
    std::cout << std::endl;

    //Now write some test msgs
    std::vector<std::string> sent_ids{ "a", "testy", "boop@7" };
    for (auto& id : sent_ids)
    {
        HLCHello test_msg;
        test_msg.source_id(id);
        test_writer.write(test_msg);
    }

    //Wait a bit, then check if the messages were actually received
    usleep(100000);
    std::lock_guard<std::mutex> lock(receive_mutex);
    for (auto& sent_id : sent_ids)
    {
        REQUIRE( std::find(received_ids.begin(), received_ids.end(), sent_id) != received_ids.end() );
    }

    REQUIRE( received_ids.size() == 3 );
}