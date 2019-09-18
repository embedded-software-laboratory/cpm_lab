#include "TrajectoryCommand.hpp"

const uint64_t dt_nanos = 100000000ull;

TrajectoryCommand::TrajectoryCommand()
:topic_vehicleCommandTrajectory(cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"))
,writer_vehicleCommandTrajectory(dds::pub::DataWriter<VehicleCommandTrajectory>(
    dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
    topic_vehicleCommandTrajectory)
)
{
    timer = std::make_shared<cpm::TimerFD>("LabControlCenter_TrajectoryCommand", dt_nanos, 0, false);

    timer->start_async([this](uint64_t t_now){
        send_trajectory(t_now);
    });
}


TrajectoryCommand::~TrajectoryCommand()
{
    timer->stop();
}

inline double vector_length(double x, double y)
{
    return sqrt(x*x+y*y);
}

void TrajectoryCommand::set_path(uint8_t vehicle_id, std::vector<Pose2D> path)
{
    if(path.size() < 3) return;
    if(timer == nullptr) return;

    /** Generate trajectory from given path **/
    std::vector<double> arc_length(path.size(), 0.0);

    for (size_t i = 1; i < path.size(); ++i)
    {
        const double dx = path.at(i).x() - path.at(i-1).x();
        const double dy = path.at(i).y() - path.at(i-1).y();
        arc_length.at(i) = arc_length.at(i-1) + vector_length(dx,dy);
    }

    const double max_speed = 1.3;
    const double max_acceleration = 0.5;

    const double standstill_time = 1.0; // time [sec] at zero speed at the beginning and end
    const double acceleration_time = max_speed / max_acceleration;
    const double deceleration_time = acceleration_time;

    const double acceleration_distance = 0.5 * max_speed * acceleration_time;
    const double deceleration_distance = 0.5 * max_speed * deceleration_time;
    const double total_distance = arc_length.back();
    const double cruise_distance = total_distance - acceleration_distance - deceleration_distance;

    if(cruise_distance < 1e-4) return; // TODO handle the case where we dont reach full speed

    const double cruise_time = cruise_distance / max_speed;
    const double total_time = standstill_time + acceleration_time + cruise_time + deceleration_time + standstill_time;

    /*std::cout 
    << "  acceleration_time  " << acceleration_time
    << "  acceleration_distance  " << acceleration_distance
    << "  deceleration_distance  " << deceleration_distance
    << "  total_distance  " << total_distance
    << "  cruise_distance  " << cruise_distance
    << "  cruise_time  " << cruise_time
    << "  total_time  " << total_time
    << std::endl;*/


    const uint64_t t_start = timer->get_time() + 1000000000ull;

    vector<TrajectoryPoint> trajectory;
    for (uint64_t t_nanos = 0; (t_nanos * 1e-9) < total_time; t_nanos += dt_nanos)
    {
        double t_sec = t_nanos * 1e-9;

        double node_distance = 0;
        double node_speed = 0;


        t_sec -= standstill_time;

        if(t_sec > 0)
        {
            const double dt = fmin(t_sec, acceleration_time);
            node_distance = 0.5 * dt * dt * max_acceleration;
            node_speed = dt * max_acceleration;
        }

        t_sec -= acceleration_time;

        if(t_sec > 0)
        {
            const double dt = fmin(t_sec, cruise_time);
            node_distance += max_speed * dt;
            node_speed = max_speed;
        }

        t_sec -= cruise_time;

        if(t_sec > 0)
        {
            const double dt = fmin(t_sec, deceleration_time);
            node_speed = max_speed - dt * max_acceleration;
            node_distance += 0.5 * dt * (node_speed + max_speed);
        }

        t_sec -= deceleration_time;


        if(t_sec > 0)
        {
            node_speed = 0;
            node_distance = total_distance;
        }


        // find path index based on distance
        int path_index = 0;
        for (int i = 1; i < path.size(); ++i)
        {
            if(fabs(arc_length.at(i) - node_distance) < fabs(arc_length.at(path_index) - node_distance))
            {
                path_index = i;
            }
        }

        TrajectoryPoint p;
        p.px(path.at(path_index).x());
        p.py(path.at(path_index).y());
        p.vx(node_speed * cos(path.at(path_index).yaw()));
        p.vy(node_speed * sin(path.at(path_index).yaw()));
        p.t().nanoseconds(t_start + t_nanos);
        trajectory.push_back(p);
    }

    std::lock_guard<std::mutex> lock(_mutex);
    this->vehicle_trajectories[vehicle_id] = trajectory;
}


void TrajectoryCommand::stop(uint8_t vehicle_id)
{
    std::lock_guard<std::mutex> lock(_mutex);
    vehicle_trajectories.erase(vehicle_id);
}


void TrajectoryCommand::stop_all()
{
    std::lock_guard<std::mutex> lock(_mutex);
    vehicle_trajectories.clear();
}




void TrajectoryCommand::send_trajectory(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(_mutex);

    for(const auto& entry : vehicle_trajectories) {
        const auto vehicle_id = entry.first;
        const auto& trajectory = entry.second;


        for (size_t i = 0; i < trajectory.size(); ++i)
        {
            // find active trajectory point
            if(t_now + 1500000000ull < trajectory.at(i).t().nanoseconds())
            {
                auto trajectoryPoint = trajectory.at(i);
                VehicleCommandTrajectory command;
                command.vehicle_id(vehicle_id);
                command.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectoryPoint));
                writer_vehicleCommandTrajectory.write(command);
                break;
            }

            // TODO delete trajectory if it is in the past
        }

    }
}