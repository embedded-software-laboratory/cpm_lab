#include "catch.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"

TEST_CASE( "VehicleIDFilteredTopic" ) {

    auto participant = cpm::ParticipantSingleton::Instance();
    dds::topic::Topic<VehicleState> topic_vehicle_state(participant, "my_topic_name");

    // sender
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicle_state);

    // receiver
    VehicleIDFilteredTopic<VehicleState> topic_vehicle42_state(topic_vehicle_state, 42);
    VehicleIDFilteredTopic<VehicleState> topic_vehicle11_state(topic_vehicle_state, 11);
    dds::sub::DataReader<VehicleState> reader_vehicle42(dds::sub::Subscriber(participant), topic_vehicle42_state);
    dds::sub::DataReader<VehicleState> reader_vehicle11(dds::sub::Subscriber(participant), topic_vehicle11_state);



    // send
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


    // wait for 'transmission'
    usleep(100000);

    // receive
    auto reader_samples11 = reader_vehicle11.take();
    auto reader_samples42 = reader_vehicle42.take();

    REQUIRE( reader_samples11.length() == 1 );
    REQUIRE( reader_samples42.length() == 1 );

    REQUIRE( reader_samples42[0].data().vehicle_id() == 42 );
    REQUIRE( reader_samples42[0].data().odometer_distance() == 2 );

    REQUIRE( reader_samples11[0].data().vehicle_id() == 11 );
    REQUIRE( reader_samples11[0].data().odometer_distance() == 3 );
}