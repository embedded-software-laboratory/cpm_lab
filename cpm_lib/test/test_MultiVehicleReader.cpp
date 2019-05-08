#include "catch.hpp"
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/stamp_message.hpp"

#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>

#include <vector>
#include <map>

TEST_CASE( "MultiVehicleReader" ) {

    auto participant = cpm::ParticipantSingleton::Instance();
    dds::topic::Topic<VehicleState> topic_vehicle_state(participant, "asldkjfhslakdj");

    // sender
    dds::pub::DataWriter<VehicleState> writer(dds::pub::Publisher(participant), topic_vehicle_state);

    // receiver
    std::vector<uint8_t> vehicle_ids{1, 3, 7};
    cpm::MultiVehicleReader<VehicleState> reader(topic_vehicle_state, vehicle_ids);


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