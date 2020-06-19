#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/ObstacleSimulationData.hpp"

#include "ObstacleSimulation.hpp"

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "CommonroadObstacleList.hpp"
#include "VehicleCommandTrajectory.hpp"

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

    //Timing
    bool use_simulated_time;
    std::string node_id;
    uint64_t dt_nanos;
    uint64_t time_step_size; //Defined by Commonroad scenario, here translated from seconds to nanoseconds
    uint64_t start_time;
    std::shared_ptr<cpm::Timer> simulation_timer;
    std::shared_ptr<cpm::SimpleTimer> standby_timer;

    //DDS
    dds::pub::DataWriter<CommonroadObstacleList> writer_commonroad_obstacle;
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicle_trajectory;

    /**
     * \brief Function that sets up the obstacle simulation based on the currently set scenario (callback for scenario)
     */
    void setup();

    /**
     * \brief Reset the simulation (callback for scenario)
     */
    void reset();

    //Stop cpm timers, if they are running
    void stop_timers();

    /**
     * \brief Create an obstacle simulation object given obstacle simulation data
     * \param id ID (set in commonroad scenario) of the obstacle
     * \param data An obstacle simulation data object containing all information relevant for simulating the translated object
     */
    void create_obstacle_simulation(int id, ObstacleSimulationData& data);

    //Send initial state of all simulation objects (when sim. is not running, to show initial position in MapView)
    void send_init_states();

    //Compute next states of commonroad obstacles based on the current time and return them
    std::vector<CommonroadObstacle> compute_all_next_states(uint64_t t_now);

public:
    /**
     * \brief Constructor to set up the simulation object
     * \param _scenario Data object to get the obstacle's data
     */
    ObstacleSimulationManager(std::shared_ptr<CommonRoadScenario> _scenario, bool use_simulated_time);

    //Destructor for threads & timer
    ~ObstacleSimulationManager();

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
};