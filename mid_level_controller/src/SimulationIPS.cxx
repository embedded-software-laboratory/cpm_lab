#include "SimulationIPS.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/stamp_message.hpp"
#include <stdlib.h>
#include <cmath>


static inline double frand() { return (double(rand()))/RAND_MAX; }
static inline double frand_sym() { return frand()*2-1; }

SimulationIPS::SimulationIPS(dds::topic::Topic<VehicleObservation>& _topic_vehicleObservation)
:topic_vehicleObservation(_topic_vehicleObservation)
,writer_vehicleObservation(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), topic_vehicleObservation)
{
    
}



void SimulationIPS::update(VehicleObservation simulatedState)
{
    // Simulate probability of detection
    if( rand()%20 == 1 ) return;


    // simulate signal noise
    simulatedState.pose().x(simulatedState.pose().x() + 0.002 * frand_sym());
    simulatedState.pose().y(simulatedState.pose().y() + 0.002 * frand_sym());
    simulatedState.pose().yaw(simulatedState.pose().yaw() + 0.01 * frand_sym());

    simulatedState.pose().yaw(remainder(simulatedState.pose().yaw(), 2*M_PI)); // yaw in range [-PI, PI]


    delay_buffer.push_back(simulatedState);


    // Send old sample
    if(delay_buffer.size() > 3)
    {
        writer_vehicleObservation.write(delay_buffer.front());
        delay_buffer.pop_front();
    }
}