#include "catch.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"

/**
 * Tests:
 * - The filter for vehicle IDs
 * - First sends data for various IDs, then checks if they were received as expected
 */

TEST_CASE( "VehicleIDFilteredTopic" ) {

    auto participant = cpm::ParticipantSingleton::Instance();
    dds::topic::Topic<VehicleState> topic_vehicle_state(participant, "my_topic_name");

    // One writer for all vehicle state test packages
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicle_state);

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