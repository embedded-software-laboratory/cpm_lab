#include "ObstacleSimulationManager.hpp"

ObstacleSimulationManager::ObstacleSimulationManager(std::shared_ptr<CommonRoadScenario> _scenario) 
:
scenario(_scenario)
{
     //Must be defined within the scenario according to specs - gives value (in seconds) of one time step unit
    double time_step_size = scenario->get_time_step_size();

    //TODO: Part for real participant: Send trajectory
    //TOOD: Set up simulated obstacles
    //TODO: Put more information in trajectory: Need to know if they are based on exact or inexact values (IntervalOrExact) for visualization
}


void ObstacleSimulationManager::set_time_scale(double scale)
{

}

void ObstacleSimulationManager::start()
{
    for (auto& entry : simulated_obstacles)
    {
        entry.start();
    }
}

void ObstacleSimulationManager::stop()
{
    for (auto& entry : simulated_obstacles)
    {
        entry.stop();
    }
}

void ObstacleSimulationManager::reset()
{
    for (auto& entry : simulated_obstacles)
    {
        entry.reset();
    }
}
