#pragma once
#include <dds/pub/ddspub.hpp>
#include "VehicleObservation.hpp"

class SimulationIPS
{

    dds::topic::Topic<VehicleObservation> topic_vehicleObservation;

public:
    SimulationIPS(dds::topic::Topic<VehicleObservation>& _topic_vehicleObservation);

    void update(VehicleObservation simulatedState);
    
};