#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"

#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <dds/pub/ddspub.hpp>

/**
 * \brief This class simulates a traffic participant / obstacle logic based on the obstacle type(s) defined in a commonroad scenario
 * It sends trajectories/... defined in the scenario (which may define position, time, velocity...)
 * These are received by either a real vehicle or a special simulated participant, that also gets a starting position etc
 */
class CommonroadObstacleSim
{
private:
    std::shared_ptr<CommonRoadScenario> scenario; //Data object that can be used to access the obstacle's trajectories

    //cpm and DDS data / objects
    bool enable_simulated_time;
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory;
    std::mutex writer_mutex;

    /**
     * \class SimulatedObstacle
     * \brief Nested class responsible for simulating a single obstacle
     * Objects of this class have access to the writer / other members, e.g. the writer
     */
    class SimulatedObstacle
    {
    private: 
        uint8_t obstacle_id;
        std::vector<TrajectoryPoint> trajectory;
        size_t current_trajectory = 0;
        uint64_t start_time;
        std::shared_pointer<cpm::Timer> timer;

    public:
        SimulatedObstacle(std::vector<TrajectoryPoint> _trajectory, int id);

        void start();
        void stop();
        void reset();
    };

    std::vector<SimulatedObstacle> simulated_obstacles;

public:
    /**
     * \brief Constructor to set up the simulation object
     * \param _scenario Data object to get the obstacle's data
     */
    CommonroadObstacleSim(std::shared_ptr<CommonRoadScenario> _scenario);

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
     * \brief Reset the simulation (callback for UI)
     */
    void reset();
};