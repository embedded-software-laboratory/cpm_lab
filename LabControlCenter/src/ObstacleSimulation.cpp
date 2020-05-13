#include "ObstacleSimulation.hpp"

ObstacleSimulation::ObstacleSimulation(std::vector<CommonTrajectoryPoint> _trajectory, double _time_step_size, int _id, bool _simulated_time)
:
writer_vehicleState(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<VehicleState>("vehicleState")),
trajectory(_trajectory),
simulated_time(_simulated_time)
{
    //Set up cpm values (cpm init has already been done before)
    node_id = "obstacle_simulation"; //Will probably not be used, as main already set LabControlCenter

    //Translate time distance to nanoseconds
    //We expect given step size to be defined in seconds
    time_step_size = _time_step_size * 1e9;
    obstacle_id = static_cast<uint8_t>(_id % 255);
    dt_nanos = time_step_size;

    //We do not accept empty trajectories
    assert(trajectory.size() > 0);
}

ObstacleSimulation::~ObstacleSimulation()
{
    if (timer)
    {
        timer->stop();
    }
    timer.reset();
}

void ObstacleSimulation::start()
{
    //Create timer here, if we do it at reset we might accidentally receive stop signals in between (-> unusable then)
    timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, simulated_time);

    //Remember start time, so that we can check how much time has passed / which state to choose when
    start_time = timer->get_time();

    timer->start_async([=] (uint64_t t_now) {
        // if (t_now - start_time > trajectory.at(trajectory.size() - 1).time.get_mean())
        // {
        //     timer->stop();
        // }

        //We must be able to use time.value(), as it is a required field
        assert(trajectory.at(current_trajectory).time.has_value());

        while (t_now - start_time > trajectory.at(current_trajectory).time.value().get_mean() * time_step_size && current_trajectory < trajectory.size() - 1)
        {
            ++current_trajectory;
        }

        //Create current state from current time, get current trajectory point - TODO: More than trivial point-hopping
        VehicleState state;
        state.vehicle_id(obstacle_id);
        state.IPS_update_age_nanoseconds(0);

        //Set header
        Header header;
        header.create_stamp(TimeStamp(t_now));
        header.valid_after_stamp(TimeStamp(t_now));
        state.header(header);

        //Set pose
        Pose2D pose;
        pose.x(trajectory.at(current_trajectory).position.first);
        pose.y(trajectory.at(current_trajectory).position.second);
        pose.yaw(trajectory.at(current_trajectory).orientation.value_or(0.0));
        state.pose(pose);

        //Set velocity, if it exists
        if(trajectory.at(current_trajectory).velocity.has_value())
        {
            state.speed(trajectory.at(current_trajectory).velocity.value().get_mean());
        }

        writer_vehicleState.write(state);
    });
}

void ObstacleSimulation::reset()
{
    if (timer)
    {
        timer->stop();
    }
    timer.reset();

    current_trajectory = 0;
    start_time = 0;
}