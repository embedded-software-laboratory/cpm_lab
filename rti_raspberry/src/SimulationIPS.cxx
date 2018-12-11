#include "SimulationIPS.hpp"

SimulationIPS::SimulationIPS(dds::topic::Topic<VehicleObservation>& _topic_vehicleObservation)
:topic_vehicleObservation(_topic_vehicleObservation)
{
    
}



void SimulationIPS::update(VehicleObservation simulatedState)
{
    // TODO add random delay and signal noise
}