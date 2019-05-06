#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "VehicleObservation.hpp"



int main(int argc, char* argv[])
{
    std::map<uint8_t, VehicleObservation> vehicleObservations;
    std::mutex vehicleObservations_mutex;

    cpm::AsyncReader<VehicleObservation> vehicleObservations_reader(
        [&](dds::sub::LoanedSamples<VehicleObservation>& samples) {
            std::unique_lock<std::mutex> lock(vehicleObservations_mutex);
            for(auto sample : samples) 
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    vehicleObservations[data.vehicle_id()] = data;
                }
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<VehicleObservation>("vehicleObservation")
    );

    return 0;
}