#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/ObstacleSimulationData.hpp"

#include "ObstacleSimulation.hpp"

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"
#include "CommonroadObstacleList.hpp"
#include "VehicleCommandTrajectory.hpp"

#include "ui/commonroad/ObstacleToggle.hpp" //For callback from vehicle toggle: Need enum defined here

#include <map>

/**
 * \brief This class simulates a traffic participant / obstacle logic based on the obstacle type(s) defined in a commonroad scenario.
 * It sends trajectories/... defined in the scenario (which may define position, time, velocity...).
 * These are received by either a real vehicle or a special simulated participant, that also gets a starting position etc.
 * Obstacles or trajectories are also drawn on the MapView. During simulation, obstacle poses are updated, else only the initial positions are drawn.
 * \ingroup lcc
 */
class ObstacleSimulationManager
{
private:
    //! Data object that can be used to access the obstacle's trajectories
    std::shared_ptr<CommonRoadScenario> scenario;

    //! Store obstacles by ID
    std::map<int, ObstacleSimulation> simulated_obstacles;
    //! Simulation state - relevant to decide which data to send for this obstacle (DDS Obstacle / Trajectory / Nothing)
    std::map<int, ObstacleToggle::ToggleState> simulated_obstacle_states;
    //! Mutex for access to the maps
    std::mutex map_mutex;

    //Timing
    //! Whether simulated time should be used for the timer that is responsible for sending obstacle trajectories or states for the MapView of the LCC
    bool use_simulated_time;
    //! Timer identifier
    std::string node_id;
    //! Timer periodicity, depends on the obstacle's periodicity
    uint64_t dt_nanos;
    //! Min. periodicity defined by Commonroad scenario, as size for each time step, here translated from seconds to nanoseconds - Commonroad time uses integers, so the smallest possible step value is 1*time_step_size
    uint64_t time_step_size;
    //! Timer for the simulation, where we need higher accuracy
    std::shared_ptr<cpm::Timer> simulation_timer;
    //! Timer for standby, that sends obstacle's initial states s.t. they are drawn on the MapView - is not called often & thus would take long to be quit if a normal Timer instead of SimpleTimer would be used
    std::shared_ptr<cpm::SimpleTimer> standby_timer;

    //! DDS writer to send obstacle information to the MapView (and potentially other participants in the network)
    cpm::Writer<CommonroadObstacleList> writer_commonroad_obstacle;
    //! DDS writer to send obstacle trajectories e.g. to a vehicle, s.t. it can follow this trajectory to represent the object in the real world
    cpm::Writer<VehicleCommandTrajectory> writer_vehicle_trajectory;

    /**
     * \brief Function that sets up the obstacle simulation based on the currently set scenario (callback for scenario)
     */
    void setup();

    /**
     * \brief Reset the simulation (callback for scenario)
     */
    void reset();

    /**
     * \brief Stop cpm timers, if they are running, and send empty trajectories to overwrite / kill existing ones (required due to preview button)
     */
    void stop_timers();

    /**
     * \brief Create an obstacle simulation object given obstacle simulation data
     * \param id ID (set in commonroad scenario) of the obstacle
     * \param data An obstacle simulation data object containing all information relevant for simulating the translated object
     */
    void create_obstacle_simulation(int id, ObstacleSimulationData& data);

    /**
     * \brief Send initial state of all simulation objects (when sim. is not running, to show initial position in MapView)
     */
    void send_init_states();

    /**
     * \brief Compute next states of commonroad obstacles based on the current time and return them; only consider obstacles that are supposed to be simulated by the LCC
     * \param t_now Current time
     * \param start_time Time the simulation was started, to compute diff to t_now
     */
    std::vector<CommonroadObstacle> compute_all_next_states(uint64_t t_now, uint64_t start_time);

    /**
     * \brief Either returns the content of the map or the default value (simulated); does not lock, so lock before calling!
     * \param id The obstacle's ID
     */
    ObstacleToggle::ToggleState get_obstacle_simulation_state(int id);

    //Helper functions
    /**
     * \brief Internal helper function that is used both for start() and start_preview()
     * \param wait_for_start_signal If the simulation should wait for a start signal
     * \param simulated_time If simulated time should be used
     */
    void start_helper(bool wait_for_start_signal, bool simulated_time);

public:
    /**
     * \brief Constructor to set up the simulation object
     * \param _scenario Data object to get the obstacle's data
     * \param use_simulated_time If simulated time should be used
     */
    ObstacleSimulationManager(std::shared_ptr<CommonRoadScenario> _scenario, bool use_simulated_time);

    /**
     * \brief Destructor for threads & timer
     */
    ~ObstacleSimulationManager();

    /**
     * \brief Scales the time used in this scenario by scale to a value in nanoseconds
     * \param scale The scale factor
     */
    void set_time_scale(double scale);

    /**
     * \brief Start the simulation preview, where we do not need a start signal (callback for UI)
     */
    void start_preview();

    /**
     * \brief Start the simulation (callback for UI)
     */
    void start();

    /**
     * \brief Stop the simulation (callback for UI)
     */
    void stop();

    /**
     * \brief Set the simulation state (off, visualized/simulated, trajectory) for an obstacle (default is simulated)
     * \param id ID of the obstacle in commonroad
     * \param state New state of the simulation
     */
    void set_obstacle_simulation_state(int id, ObstacleToggle::ToggleState state);
};