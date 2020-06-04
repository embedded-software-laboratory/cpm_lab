#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"

#include "ObstacleSimulation.hpp"

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "SystemTrigger.hpp"

/**
 * \brief This class simulates a traffic participant / obstacle logic based on the obstacle type(s) defined in a commonroad scenario
 * It sends trajectories/... defined in the scenario (which may define position, time, velocity...)
 * These are received by either a real vehicle or a special simulated participant, that also gets a starting position etc
 */
class ObstacleSimulationManager
{
private:
    std::shared_ptr<CommonRoadScenario> scenario; //Data object that can be used to access the obstacle's trajectories

    std::vector<ObstacleSimulation> simulated_obstacles;

    double time_step_size;
    bool use_simulated_time;

    //DDS, to send a custom stop signal to the simple timers of the simulated obstacles
    dds::pub::DataWriter<SystemTrigger> writer_stop_signal;
    uint64_t custom_stop_signal_diff = 1;

    /**
     * \brief Function that sets up the obstacle simulation based on the currently set scenario (callback for scenario)
     */
    void setup();

    /**
     * \brief Reset the simulation (callback for scenario)
     */
    void reset();

public:
    /**
     * \brief Constructor to set up the simulation object
     * \param _scenario Data object to get the obstacle's data
     */
    ObstacleSimulationManager(std::shared_ptr<CommonRoadScenario> _scenario, bool use_simulated_time);

    /**
     * \brief Scales the time used in this scenario by scale to a value in nanoseconds
     * \param scale The scale factor
     */
    void set_time_scale(double scale);

    /**
     * \brief Start the simulation (callback for UI)
     */
    void start();

    /**
     * \brief Stop the simulation (callback for UI)
     */
    void stop();

    /**
     * \brief Send a stop signal to the simple timers of the simulated obstacles that run when no simulation is performed
     */
    void send_stop_signal();
};