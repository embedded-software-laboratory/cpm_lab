#pragma once
#include <dds/pub/ddspub.hpp>
#include "VehicleObservation.hpp"
#include <list>

class SimulationIPS
{

    dds::topic::Topic<VehicleObservation> topic_vehicleObservation;
    dds::pub::DataWriter<VehicleObservation> writer_vehicleObservation;

    std::list<VehicleObservation> delay_buffer;

public:
    SimulationIPS(dds::topic::Topic<VehicleObservation>& _topic_vehicleObservation);

    void update(VehicleObservation simulatedState);
    
};