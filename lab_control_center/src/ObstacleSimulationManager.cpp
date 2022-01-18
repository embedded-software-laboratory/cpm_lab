#include "ObstacleSimulationManager.hpp"

/**
 * \file ObstacleSimulationManager.cpp
 * \ingroup lcc
 */

ObstacleSimulationManager::ObstacleSimulationManager(std::shared_ptr<CommonRoadScenario> _scenario, bool _use_simulated_time) 
:
scenario(_scenario),
use_simulated_time(_use_simulated_time),
writer_commonroad_obstacle("commonroadObstacle"),
writer_vehicle_trajectory("vehicleCommandTrajectory")
{
    //Set up cpm values (cpm init has already been done before)
    node_id = "obstacle_simulation"; //Will probably not be used, as main already set LabControlCenter

    //Warning: Do not set up the simulation manager when multiple threads are already running, or you risk that during construction the scenario gets changed!
    //The scenario callbacks are always called with locked mutexes within the scenario, so there is no need to worry about them afterwards
    setup();

    //Register at scenario to get reset when a new scenario is loaded
    //This makes sure that all data structures can be used if reset() was not called, and are reset in the right order otherwise
    scenario->register_obstacle_sim(
        [&] ()
        {
            setup();
        },
        [&] ()
        {
            reset();
        }
    );
}

ObstacleSimulationManager::~ObstacleSimulationManager()
{
    stop_timers();
}


void ObstacleSimulationManager::stop_timers()
{
    std::cout << "Stopping sim timer" << std::endl;
    if (simulation_timer)
    {
        simulation_timer->stop();
    }
    simulation_timer.reset();

    std::cout << "Stopping standby timer" << std::endl;
    if (standby_timer)
    {
        standby_timer->stop();
    }
    standby_timer.reset();

    //Send empty trajectories for all obstacles that are in trajectory mode
    //Might not work as desired in case of simulated time, but that does not really
    //matter because the preview itself would not work for simulated time
    auto t_now = cpm::get_time_ns();
    VehicleCommandTrajectory trajectory;
    Header header;
    header.create_stamp(TimeStamp(t_now));
    header.valid_after_stamp(TimeStamp(t_now));
    trajectory.header(header);
    
    std::lock_guard<std::mutex> lock(map_mutex);
    for (auto& obstacle : simulated_obstacles)
    {
        if (get_obstacle_simulation_state(obstacle.second.get_id()) == ObstacleToggle::ToggleState::On) //TODO: Let the user choose which obstacle should be real in the UI
        {
            trajectory.vehicle_id(obstacle.second.get_id());
            writer_vehicle_trajectory.write(trajectory);
        }
    }
}

void ObstacleSimulationManager::send_init_states()
{
    //Send initial states with slow timer (do not need to send often in this case) - send, but less frequently, to make sure that everyone gets this data
    //Sending once at the right time would be sufficient as well, but this should not take up much computation time / energy
    standby_timer = std::make_shared<cpm::SimpleTimer>(node_id, 1000ull, false, false);
    //uint64_t time_step_size_ns = 1000ull * 1e6;
    standby_timer->start_async([&] (uint64_t t_now) {
        std::lock_guard<std::mutex> lock(map_mutex);

        //Get and send initial states
        std::vector<CommonroadObstacle> initial_obstacle_states;
        for (auto& obstacle : simulated_obstacles)
        {
            //if (get_obstacle_simulation_state(obstacle.second.get_id()) == ObstacleToggle::ToggleState::Simulated)
            //{
                initial_obstacle_states.push_back(obstacle.second.get_init_state(t_now));
            //}
        }

        CommonroadObstacleList obstacle_list;
        obstacle_list.commonroad_obstacle_list(initial_obstacle_states);
        writer_commonroad_obstacle.write(obstacle_list);

        //Send test init. trajectory messages
        // for (auto& obstacle : simulated_obstacles)
        // {
        //     if (get_obstacle_simulation_state(obstacle.second.get_id()) == ObstacleToggle::ToggleState::On) //TODO: Let the user choose which obstacle should be real in the UI
        //     {
        //         writer_vehicle_trajectory.write(obstacle.second.get_init_trajectory(t_now, time_step_size_ns));
        //     }
        // }
    });
}

std::vector<CommonroadObstacle> ObstacleSimulationManager::compute_all_next_states(uint64_t t_now, uint64_t start_time)
{
    //TODO: Thread pool?
    std::vector<CommonroadObstacle> next_obstacle_states;
    std::lock_guard<std::mutex> lock(map_mutex);

    for (auto& obstacle : simulated_obstacles)
    {
        //Only simulate obstacles that are supposed to be simulated
        if (get_obstacle_simulation_state(obstacle.second.get_id()) == ObstacleToggle::ToggleState::Simulated)
        {
            next_obstacle_states.push_back(obstacle.second.get_state(start_time, t_now, time_step_size));
        }
        
    }

    return next_obstacle_states;
}

void ObstacleSimulationManager::setup()
{
    //Translate time distance to nanoseconds
    //We expect given step size to be defined in seconds (gives value of one time step unit, must be defined according to specs)
    time_step_size = static_cast<uint64_t>(scenario->get_time_step_size() * 1e9);
    //Restrict period callback to at most 20ms
    dt_nanos = 20000000ull;
    if (time_step_size < 20000000ull)
    {
        dt_nanos = time_step_size;
    }

    //Set up simulated obstacles
    auto dynamic_obstacle_ids = scenario->get_dynamic_obstacle_ids();
    for (auto obstacle_id : dynamic_obstacle_ids)
    {
        auto obstacle = scenario->get_dynamic_obstacle(obstacle_id);
        if (!obstacle.has_value())
        {
            //We encountered an error that should not have happened unless the commonroad object was changed during setup - this should never happen though
            throw std::runtime_error("Could not set up obstacle simulation manager due to wrong ID or change of scenario during setup (which should not be possible)!");
        }

        auto obstacle_data = obstacle.value().get_obstacle_simulation_data();
        create_obstacle_simulation(obstacle_id, obstacle_data);
    }

    auto static_obstacle_ids = scenario->get_static_obstacle_ids();
    for (auto obstacle_id : static_obstacle_ids)
    {
        auto obstacle = scenario->get_static_obstacle(obstacle_id);
        if (!obstacle.has_value())
        {
            //We encountered an error that should not have happened unless the commonroad object was changed during setup - this should never happen though
            throw std::runtime_error("Could not set up obstacle simulation manager due to wrong ID or change of scenario during setup (which should not be possible)!");
        }

        auto obstacle_data = obstacle.value().get_obstacle_simulation_data();
        create_obstacle_simulation(obstacle_id, obstacle_data);
    }

    auto environment_obstacle_ids = scenario->get_environment_obstacle_ids();
    for (auto obstacle_id : environment_obstacle_ids)
    {
        auto obstacle = scenario->get_environment_obstacle(obstacle_id);
        if (!obstacle.has_value())
        {
            //We encountered an error that should not have happened unless the commonroad object was changed during setup - this should never happen though
            throw std::runtime_error("Could not set up obstacle simulation manager due to wrong ID or change of scenario during setup (which should not be possible)!");
        }

        auto obstacle_data = obstacle.value().get_obstacle_simulation_data();
        create_obstacle_simulation(obstacle_id, obstacle_data);
    }

    send_init_states();

    //TODO: Part for real participant: Send trajectory
    //TODO: Put more information in trajectory: Need to know if they are based on exact or inexact values (IntervalOrExact) for visualization
}

void ObstacleSimulationManager::create_obstacle_simulation(int id, ObstacleSimulationData& data)
{
    //We need to modify the trajectory first: Lanelet refs need to be translated to a trajectory
    for (auto& point : data.trajectory)
    {
        if (point.lanelet_ref.has_value())
        {
            //Override shape with lanelet reference shape, as there is no positional value more exact than the whole lanelet anyway (represent that the object could be anywhere on it)
            //TODO: Does the shape need to be within the lanelet, or can e.g. its center go up to its borders?
            CommonroadDDSPolygon lanelet_polygon;

            auto lanelet = scenario->get_lanelet(point.lanelet_ref.value());
            if (lanelet.has_value())
            {
                auto lanelet_points = lanelet->get_shape();
                std::vector<CommonroadDDSPoint> dds_lanelet_points;
                for (auto lanelet_point : lanelet_points)
                {
                    dds_lanelet_points.push_back(lanelet_point.to_dds_msg());
                }
                lanelet_polygon.points(dds_lanelet_points);

                CommonroadDDSShape shape;
                std::vector<CommonroadDDSPolygon> polygons;
                polygons.push_back(lanelet_polygon);
                shape.polygons(polygons);

                point.shape = shape;
            }
        }
    }

    std::lock_guard<std::mutex> lock(map_mutex);
    simulated_obstacles.emplace(id, ObstacleSimulation(data, id));
}

//Suppress warning for unused parameter
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void ObstacleSimulationManager::set_time_scale(double scale)
{
    //TODO: If desired, adjust time scale
}
#pragma GCC diagnostic pop

void ObstacleSimulationManager::start_helper(bool wait_for_start_signal, bool simulated_time)
{
    stop_timers();

    //Create simulation_timer here, if we do it at reset we might accidentally receive stop signals in between (-> unusable then)
    simulation_timer = cpm::Timer::create(node_id, dt_nanos, 0, wait_for_start_signal, true, simulated_time);

    std::cout << std::endl << std::endl << "--------- Set time step size to: " << time_step_size << std::endl << std::endl;

    simulation_timer->start_async([&] (uint64_t t_now) {
        //Cannot be obtained before the timer was started
        auto start_time = simulation_timer->get_start_time();
        
        auto next_obstacle_states = compute_all_next_states(t_now, start_time);

        CommonroadObstacleList obstacle_list;
        obstacle_list.commonroad_obstacle_list(next_obstacle_states);
        writer_commonroad_obstacle.write(obstacle_list);

        //Send test trajectory messages
        std::lock_guard<std::mutex> lock(map_mutex);
        for (auto& obstacle : simulated_obstacles)
        {
            if (get_obstacle_simulation_state(obstacle.second.get_id()) == ObstacleToggle::ToggleState::On) //TODO: Let the user choose which obstacle should be real in the UI
            {
                writer_vehicle_trajectory.write(obstacle.second.get_trajectory(start_time, t_now, time_step_size));
            }
        }
    });
}

void ObstacleSimulationManager::start_preview()
{
    start_helper(false, false);
}

void ObstacleSimulationManager::start()
{
    start_helper(true, use_simulated_time);
}

void ObstacleSimulationManager::stop()
{
    stop_timers();

    std::lock_guard<std::mutex> lock(map_mutex);
    for (auto& simulated_obstacle : simulated_obstacles)
    {
        simulated_obstacle.second.reset();
    }

    send_init_states();
}

void ObstacleSimulationManager::reset()
{
    stop_timers();

    std::lock_guard<std::mutex> lock(map_mutex);
    simulated_obstacles.clear();
    simulated_obstacle_states.clear();
}

void ObstacleSimulationManager::set_obstacle_simulation_state(int id, ObstacleToggle::ToggleState state)
{
    std::lock_guard<std::mutex> lock(map_mutex);
    simulated_obstacle_states[id] = state;

    //TODO: Maybe use mutex
}

ObstacleToggle::ToggleState ObstacleSimulationManager::get_obstacle_simulation_state(int id)
{
    auto element = simulated_obstacle_states.find(id);
    if (element != simulated_obstacle_states.end())
    {
        return element->second;
    }
    return ObstacleToggle::ToggleState::Simulated;
}