#pragma once
#include <stdint.h>
#include "defaults.hpp"
#include "Pose2D.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/TimerFD.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"

/**
 * \class TrajectoryCommand
 * \brief Using this class, trajectories from 2D paths can be created and sent to the vehicles. 
 * Used to create and send trajectories from paths drawn in the MapView
 * \ingroup lcc
 */
class TrajectoryCommand
{
    //! Mutex for accessing vehicle_trajectories
    std::mutex _mutex;
    //! Map that stores vehicle trajectories for multiple vehicles (vehicle ID as identifier, uint8_t)
    map<uint8_t, vector<TrajectoryPoint>> vehicle_trajectories;
    /**
     * \brief Timer to regularly send new trajectory data to the vehicles (may send nothing in between, if no such data exists). 
     * Calls send_trajectory in the callback. 
     * Does not respond to stop signals as it can be used independt of running simulations 
     * (as vehicle paths can always be drawn, also to align vehicles before a simulation).
     */
    std::shared_ptr<cpm::TimerFD> timer;

    //! Writer to send trajectories to the vehicles
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory;

    /**
     * \brief Function to send a current trajectory, created from the drawn path in the LCC's MapView using set_path and stored
     * in vehicle_trajectories. Takes the current time as input to determine which part of the trajectory to send.
     * May not send anything if currently no new trajectories exist.
     * 
     * \param t_now Current time in nanoseconds, as given by the timer
     */
    void send_trajectory(uint64_t t_now);

public:
    /**
     * \brief Constructor, sets up the writer, starts the timer (async. / in a new thread internally)
     */
    TrajectoryCommand();

    /**
     * \brief Stops the timer (thread)
     */
    ~TrajectoryCommand();

    /**
     * \brief Translate a path for a vehicle to a trajectory for the same vehicle and store it in vehicle_trajectories
     * \param vehicle_id The vehicle to create the trajectory points for
     * \param path The path drawn for the vehicle in the LCC's MapView
     */
    void set_path(uint8_t vehicle_id, std::vector<Pose2D> path);
    
    /**
     * \brief Translate a path for a vehicle to a trajectory for the same vehicle and store it in vehicle_trajectories
     * \param vehicle_id The vehicle to create the trajectory points for
     * \param path A path consisting of poses
     * \param delay_ns A delay in nanoseconds before the trajectory messsages are sent
     * \return The duration of the trajectory in nanoseconds
     */
    uint64_t set_path(uint8_t vehicle_id, std::vector<Pose2D> path, uint64_t delay_ns);

    /**
     * \brief Erases all created vehicle trajectories in vehicle_trajectories for the given vehicle.
     * Thus, all previously created trajectories are no longer executed and thus stopped for that vehicle.
     * \param vehicle_id The vehicle to stop the trajectory "execution" for
     */
    void stop(uint8_t vehicle_id);

    /**
     * \brief  Erases all created vehicle trajectories in vehicle_trajectories.
     * Thus, all previously created trajectories are no longer executed and thus stopped.
     */
    void stop_all();
    
};