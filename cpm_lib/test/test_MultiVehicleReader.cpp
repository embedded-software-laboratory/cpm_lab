#include "catch.hpp"
#include "cpm/dds/VehicleState.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/stamp_message.hpp"

#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"

#include <vector>
#include <map>

/**
 * \test Tests MultiVehicleReader
 * \ingroup cpmlib
 */
TEST_CASE( "MultiVehicleReader" ) {
    
    // sender
    cpm::Writer<VehicleState> writer("asldkjfhslakdj");

    // receiver
    std::vector<uint8_t> vehicle_ids{1, 3, 7};
    cpm::MultiVehicleReader<VehicleState> reader(cpm::get_topic<VehicleState>("asldkjfhslakdj"), vehicle_ids);

    const uint64_t second = 1000000000ull;
    const uint64_t millisecond = 1000000ull;
    const uint64_t t0 = 1500000000ull * second;
    const uint64_t expected_delay = 400 * millisecond;

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in MultiVehicleReader test" << std::endl << "\t";
    bool wait = true;
    while (wait)
    {
        usleep(10000); //Wait 10ms
        std::cout << "." << std::flush;

        if (writer.matched_subscriptions_size() > 0)
            wait = false;
    }
    std::cout << std::endl;

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

    // Read might need to be repeated if not all data has been received yet, so check map size
    // As the maps are cleared, we need to merge them with every read
    // Wait up to 1 second for all data
    std::map<uint8_t, VehicleState> samples;
    std::map<uint8_t, uint64_t> samples_age;
    for (int i = 0; i < 10; ++i)
    {
        //Preserve old data
        std::map<uint8_t, VehicleState> samples_temp(samples);
        std::map<uint8_t, uint64_t> samples_age_temp(samples_age);

        const uint64_t t_now = t0 + 5 * second + 300 * millisecond;
        reader.get_samples(t_now, samples, samples_age);

        //Merge all received data, new data gets priority (old data is only inserted where a key is missing)
        samples.insert(samples_temp.begin(), samples_temp.end());
        samples_age.insert(samples_age_temp.begin(), samples_age_temp.end());

        //Check if everything was received yet
        bool entries_exist = false;
        if (
            samples.find(vehicle_ids.at(0)) != samples.end() &&
            samples.find(vehicle_ids.at(1)) != samples.end() &&
            samples.find(vehicle_ids.at(2)) != samples.end()
        )
        {
            entries_exist = true;
        }

        bool entries_up_to_date = false;
        if (entries_exist)
        {
            if (samples[vehicle_ids.at(0)].odometer_distance() == 4 &&
                samples[vehicle_ids.at(1)].odometer_distance() == 5 &&
                samples[vehicle_ids.at(2)].odometer_distance() == 6
            )
            {
                entries_up_to_date = true;
            }
        }

        //Stop the loop if all entries could be found, else wait a bit more in case they just have not been received yet
        if (entries_up_to_date) break;
        else usleep(100000);
    }

    //Check if the newest data could be read
    REQUIRE( samples_age[vehicle_ids.at(0)] == 900 * millisecond );
    REQUIRE( samples[vehicle_ids.at(0)].odometer_distance() == 4 );
    REQUIRE( samples[vehicle_ids.at(0)].vehicle_id() == 1 );

    REQUIRE( samples_age[vehicle_ids.at(1)] == 900 * millisecond );
    REQUIRE( samples[vehicle_ids.at(1)].odometer_distance() == 5 );
    REQUIRE( samples[vehicle_ids.at(1)].vehicle_id() == 3 );

    REQUIRE( samples_age[vehicle_ids.at(2)] == 900 * millisecond );
    REQUIRE( samples[vehicle_ids.at(2)].odometer_distance() == 6 );
    REQUIRE( samples[vehicle_ids.at(2)].vehicle_id() == 7 );

    //The reader was set to listen to other IDs, so if data for vehicle 2 was received, something went wrong
    bool hasVehicleTwo = false;
    for (auto entry : samples) {
        if (entry.second.vehicle_id() == 2) {
            hasVehicleTwo = true;
        }
    }

    REQUIRE(!hasVehicleTwo);
}