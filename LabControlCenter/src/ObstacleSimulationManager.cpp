#include "ObstacleSimulationManager.hpp"

ObstacleSimulationManager::ObstacleSimulationManager(std::shared_ptr<CommonRoadScenario> _scenario, bool _use_simulated_time) 
:
scenario(_scenario),
use_simulated_time(_use_simulated_time),
writer_stop_signal(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<SystemTrigger>("systemTrigger"), dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable())
{
    //Warning: Do not set up the simulation manager when multiple threads are already running, or you risk that during construction the scenario gets changed!
    //The scenario callbacks are always called with locked mutexes within the scenario, so there is no need to worry about them
    setup();

    //Register at scenario to get reset when a new scenario is loaded
    //This makes sure that all data structures can be used if reset() was not called, and are reset in the right order otherwise
    scenario->register_obstacle_sim(
        [=] ()
        {
            setup();
        },
        [=] ()
        {
            reset();
        }
    );
}

void ObstacleSimulationManager::setup()
{
    //Must be defined within the scenario according to specs - gives value (in seconds) of one time step unit
    double time_step_size = scenario->get_time_step_size();

    //Set up simulated obstacles
    auto obstacle_ids = scenario->get_dynamic_obstacle_ids();
    for (auto obstacle_id : obstacle_ids)
    {
        auto trajectory = scenario->get_dynamic_obstacle(obstacle_id).value().get_trajectory();
        //We need to modify the trajectory first: Lanelet refs need to be translated to a trajectory
        for (auto& point : trajectory.trajectory)
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

        simulated_obstacles.push_back(
            ObstacleSimulation(trajectory, time_step_size, obstacle_id, use_simulated_time, cpm::TRIGGER_STOP_SYMBOL - custom_stop_signal_diff)
        );
    }

    for (auto& simulated_obstacle : simulated_obstacles)
    {
        simulated_obstacle.send_init_state();
    }

    //TODO: Part for real participant: Send trajectory
    //TODO: Put more information in trajectory: Need to know if they are based on exact or inexact values (IntervalOrExact) for visualization
}

//Suppress warning for unused parameter
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void ObstacleSimulationManager::set_time_scale(double scale)
{
    //TODO: If desired, adjust time scale
}
#pragma GCC diagnostic pop

void ObstacleSimulationManager::start()
{
    for (auto& entry : simulated_obstacles)
    {
        entry.start();
    }
}

void ObstacleSimulationManager::stop()
{
    send_stop_signal();

    ++custom_stop_signal_diff;
    //Use three kinds of custom stop signals, rotate with every stop / reset
    if (custom_stop_signal_diff > 3)
    {
        custom_stop_signal_diff = 1;
    }
    uint64_t new_stop_signal = cpm::TRIGGER_STOP_SYMBOL - custom_stop_signal_diff;

    for (auto& entry : simulated_obstacles)
    {
        entry.reset(new_stop_signal);
    }
}

void ObstacleSimulationManager::reset()
{
    send_stop_signal();

    ++custom_stop_signal_diff;
    //Use three kinds of custom stop signals, rotate with every stop / reset
    if (custom_stop_signal_diff > 3)
    {
        custom_stop_signal_diff = 1;
    }

    for (auto& entry : simulated_obstacles)
    {
        entry.stop();
    }

    simulated_obstacles.clear();
}

void ObstacleSimulationManager::send_stop_signal()
{
    SystemTrigger trigger;
    trigger.next_start(TimeStamp(cpm::TRIGGER_STOP_SYMBOL - custom_stop_signal_diff));
    writer_stop_signal.write(trigger);
}