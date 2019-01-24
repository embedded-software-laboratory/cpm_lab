#include "catch.hpp"
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/stamp_message.hpp"

#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>
#include "cpm/Reader.hpp"

TEST_CASE( "Reader" ) {

    auto participant = cpm::ParticipantSingleton::Instance();
    dds::topic::Topic<VehicleState> topic_vehicle_state(participant, "asldkjfhslakdj");

    // sender
    dds::pub::DataWriter<VehicleState> writer(dds::pub::Publisher(participant), topic_vehicle_state);

    // receiver
    cpm::Reader<VehicleState> reader(topic_vehicle_state);


    const uint64_t second = 1000000000ull;
    const uint64_t millisecond = 1000000ull;
    const uint64_t t0 = 1500000000ull * second;
    const uint64_t expected_delay = 400 * millisecond;

    // send
    for (uint64_t t_now = t0; t_now <= t0 + 10*second; t_now += second)
    {
        VehicleState vehicleState;
        vehicleState.odometer_distance( (t_now-t0)/second );
        vehicleState.vehicle_id(2);
        cpm::stamp_message(vehicleState, t_now, expected_delay);
        writer.write(vehicleState);
        usleep(10000);
    }

    sleep(1);



    // example read
    VehicleState sample;
    uint64_t sample_age;
    const uint64_t t_now = t0 + 5 * second + 300 * millisecond;
    reader.get_sample(t_now, sample, sample_age);

    REQUIRE( sample_age == 900 * millisecond );
    REQUIRE( sample.odometer_distance() == 4 );
}