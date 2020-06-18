// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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