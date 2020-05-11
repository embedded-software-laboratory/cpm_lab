#include "CommonroadObstacleSim.hpp"

CommonroadObstacleSim::CommonroadObstacleSim(std::shared_ptr<CommonRoadScenario> _scenario) 
:
scenario(_scenario)
{
    //Set up cpm library
    const std::string node_id = "basic_circle_example";
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id(node_id);
    enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    
    //Set up DDS data
    dds::pub::DataWriter<VehicleState> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleState>("vehicleState")
    );

    //TODO: Timer for each obstacle
    //TODO: Separate into two classes, of which one simulates one participant with a given trajectory, and the other manages these objects
    //TOOD: Set up simulated obstacles
    //TODO: Determine time scale for simulated obstacles (common scale? Or determine using speed / (else) use time values * scale?)
    //TODO: Show warning if distance/speed != time_diff?
}

void CommonroadObstacleSim::SimulatedObstacle::SimulatedObstacle(std::vector<TrajectoryPoint> _trajectory, int id) 
:
trajectory(_trajectory)
{
    //Set up DDS data
    obstacle_id = static_cast<uint8_t>(id % 255);
    uint64_t dt_nanos = 400000000ull; // 400 milliseconds == 400000000 nanoseconds
    auto timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, enable_simulated_time);
}

void CommonroadObstacleSim::SimulatedObstacle::start()
{
    //Remember start time, so that we can check how much time has passed / which state to choose when
    start_time = timer->get_time();

    timer->start_async([=] (uint64_t t_now) {
        if (current_trajectory >= trajectory.size())
        {
            timer->stop();
        }

        //TODO: Create current state from current time, get current trajectory point

        //TODO: Check mutex and send current state if writer is currently free (potential problem: can't use writer in too quick successions)
        //TODO: Alternative: Writer for each simulated obstacle


        //TODO: Only increase if specified time has passed (regard time scale in combination with time!)
        ++current_trajectory;
    });
}

void CommonroadObstacleSim::SimulatedObstacle::stop()
{
    timer->stop();
}

void CommonroadObstacleSim::SimulatedObstacle::reset()
{

}


void CommonroadObstacleSim::set_time_scale(double scale)
{

}

void CommonroadObstacleSim::start()
{
    for (const auto& entry : simulated_obstacles)
    {
        entry.start();
    }
}

void CommonroadObstacleSim::stop()
{
    for (const auto& entry : simulated_obstacles)
    {
        entry.stop();
    }
}

void CommonroadObstacleSim::reset()
{
    for (const auto& entry : simulated_obstacles)
    {
        entry.reset();
    }
}
