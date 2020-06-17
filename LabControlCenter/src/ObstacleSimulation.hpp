#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/ObstacleSimulationData.hpp"

#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/SimpleTimer.hpp"
#include "CommonroadObstacle.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"
#include <dds/pub/ddspub.hpp>


/**
 * \class ObstacleSimulation
 * \brief Nested class responsible for simulating a single obstacle
 * Objects of this class have access to the writer / other members, e.g. the writer
 */
class ObstacleSimulation
{
private: 
    //DDS
    dds::pub::DataWriter<CommonroadObstacle> writer_commonroad_obstacle;

    //Trajectory info
    uint8_t obstacle_id;
    ObstacleSimulationData trajectory; //Important: Position should always be set! Translate lanelet refs beforehand!
    uint64_t time_step_size;
    size_t current_trajectory = 0;

    //Timing
    bool simulated_time;
    std::string node_id;
    uint64_t dt_nanos;
    uint64_t start_time;
    std::shared_ptr<cpm::Timer> simulation_timer;
    std::shared_ptr<cpm::SimpleTimer> standby_timer;

    //Custom stop signal for timer when stopped
    uint64_t custom_stop_signal;

    /**
     * \brief Interpolation function that delivers state values in between set trajectory points
     * \return x,y,yaw values using references as input
     */
    void interpolate_between(ObstacleSimulationSegment p1, ObstacleSimulationSegment p2, double current_time, double &x_interp, double &y_interp, double &yaw_interp);
    
    //Send the current obstacle state based on the given trajectory point
    void send_state(ObstacleSimulationSegment& point, uint64_t t_now);

    void stop_timers();

public:
    /**
     * \brief constructor
     * \param _trajectory The trajectory to follow: Important: Translate lanelet ref to position beforehand, so that it must not be done here anymore (a value is expected for every single trajectory point)
     * \param _time_step_size The size of one commonroad timestep (in seconds)
     * \param _id The ID of the simulated obstacle
     * \param _simulated_time Whether simulated time should be used (TODO: Not properly supported /tested yet)
     * \param _custom_stop_signal Custom stop signal for the slow timer that is used when no simulation is performed; Can be used to stop all running obstacle simulations at once & thus to save time when switching to simulation mode
     * \param _get_lanelet_shape Function that returns shape (+ position) of a lanelet (given its ID), used when only a lanelet reference determines an obstacle's position
     */
    ObstacleSimulation(ObstacleSimulationData _trajectory, double _time_step_size, int _id, bool _simulated_time, uint64_t _custom_stop_signal);

    //Destructor for timer
    ~ObstacleSimulation();

    /**
     * \brief Send the initial state of the obstacles periodically until the simulation is started
     */
    void send_init_state();

    void start();

    /**
     * \brief Reset the obstacle simulation (show a static obstacle again)
     * \param new_custom_stop_signal Stop signal for the newly started static obstacle simulation (is changed so that outdated messages don't make the new timer stop)
     */
    void reset(uint64_t new_custom_stop_signal);

    /**
     * \brief Stop the obstacle simulation & all timers
     */
    void stop();
};