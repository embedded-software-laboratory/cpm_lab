#pragma once
#include <thread>
#include "SimulationVehicle.hpp"

class SimulationIPS
{

    std::thread m_thread;
    bool active = true;
public:
    SimulationIPS();
    ~SimulationIPS();
    
};