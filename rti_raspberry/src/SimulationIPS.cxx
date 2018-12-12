#include "SimulationIPS.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/stamp_message.hpp"
#include <stdlib.h>


static inline double frand() { return (double(rand()))/RAND_MAX; }
static inline double frand_sym() { return frand()*2-1; }

SimulationIPS::SimulationIPS(dds::topic::Topic<VehicleObservation>& _topic_vehicleObservation)
:topic_vehicleObservation(_topic_vehicleObservation)
,writer_vehicleObservation(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), topic_vehicleObservation)
{
    
}



void SimulationIPS::update(VehicleObservation simulatedState)
{
    // Simulate 25% probability of detection
    if( rand()%4 != 1 ) return;


    // simulate signal noise
    simulatedState.pose().x(simulatedState.pose().x() + 0.005 * frand_sym());
    simulatedState.pose().y(simulatedState.pose().y() + 0.005 * frand_sym());
    simulatedState.pose().yaw(simulatedState.pose().yaw() + 0.03 * frand_sym());


    delay_buffer.push_back(simulatedState);


    // Send old sample
    if(delay_buffer.size() > 10)
    {
        writer_vehicleObservation.write(delay_buffer.front());
        delay_buffer.pop_front();
    }
}