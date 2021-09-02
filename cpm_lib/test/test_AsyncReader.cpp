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
        usleep(10000); //Wait 10ms
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

    //Wait up to 1 second before checking if the messages were actually received
    //Abort waiting if all messages are already there
    for (int i = 0; i < 10; ++i)
    {
        usleep(100000);

        std::lock_guard<std::mutex> lock(receive_mutex);
        if (received_ids.size() >= 3) break;
    }

    std::lock_guard<std::mutex> lock(receive_mutex);
    for (auto& sent_id : sent_ids)
    {
        REQUIRE( std::find(received_ids.begin(), received_ids.end(), sent_id) != received_ids.end() );
    }

    REQUIRE( received_ids.size() == 3 );
}