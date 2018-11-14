#include "SimulationIPS.hpp"
#include <unistd.h>

SimulationIPS::SimulationIPS()
{
    m_thread = std::thread([&](){
        while(active)
        {
            // ...
            sleep(1);
        }
    });
}


SimulationIPS::~SimulationIPS()
{
    active = false;
    m_thread.join();
}