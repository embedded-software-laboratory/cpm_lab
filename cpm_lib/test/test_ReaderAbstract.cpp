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

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in ReaderAbstract test" << std::endl << "\t";
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
    for (size_t i = 0; i < 5; ++i)
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance( static_cast<double>(i) );
        expected_odometer_values.push_back( static_cast<double>(i) );
        sample_writer.write(vehicleState);
        usleep(10000);
    }
    
    //Read should contain desired dataexpected_odometer_values
    //Continue reading until all data has been received or 1 second has passed (to compensate test for machines with slow DDS / net. / VMs)
    std::vector<double> received_odometer_values;
    for (int i = 0; i < 10; ++i)
    {
        auto samples = reader.take();
        for (auto& sample : samples)
        {
            received_odometer_values.push_back(sample.odometer_distance());
        }

        if (received_odometer_values.size() >= 5)
            break;
        else
            usleep(100000);
    }

    for( auto expected_value : expected_odometer_values )
    {
        REQUIRE( std::find(received_odometer_values.begin(), received_odometer_values.end(), expected_value) != received_odometer_values.end() );
    }
}