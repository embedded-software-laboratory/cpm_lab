#include "TrajectoryCommand.hpp"

TrajectoryCommand::TrajectoryCommand()
:topic_vehicleCommandTrajectory(cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory"))
,writer_vehicleCommandTrajectory(dds::pub::DataWriter<VehicleCommandTrajectory>(
    dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
    topic_vehicleCommandTrajectory)
)
{
    timer = std::make_shared<cpm::TimerFD>("LabControlCenter_TrajectoryCommand",40000000ull, 0, false);

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


    /********** Generate trajectory from given path ********/







    

    vector<TrajectoryPoint> trajectory;
    /*int path_index = 0;
    for (int path_index = 0; path_index < path.size(); ++path_index)
    {
        TrajectoryPoint p;
        p.px(path.at(path_index).x);
        p.py(path.at(path_index).y);
        trajectory.push_back(p);
        path_index++;
    }

    // Set directions and speed profile for the trajectory
    for (size_t i = 1; i < trajectory.size()-1; ++i)
    {
        double direction_x = trajectory.at(i+1).px() - trajectory.at(i-1).px();
        double direction_y = trajectory.at(i+1).py() - trajectory.at(i-1).py();
        double direction_length = vector_length(direction_x, direction_y);
        direction_x /= direction_length;
        direction_y /= direction_length;

        double reference_speed = 1.0;
        if(i == 1 || i == trajectory.size()-2) 
        {
            reference_speed = 0.7;
        }

        trajectory.at(i).vx(direction_x * reference_speed);
        trajectory.at(i).vy(direction_y * reference_speed);
    }

    // Set the timing of the trajectory nodes
    uint64_t t_start = timer->get_time() + 2000000000ull;
    trajectory.at(0).t().nanoseconds(t_start);

    for (size_t i = 1; i < trajectory.size(); ++i)
    {
        const double segment_length = vector_length(
            trajectory.at(i).px() - trajectory.at(i-1).px(), 
            trajectory.at(i).py() - trajectory.at(i-1).py()
        );
        const double speed1 = vector_length(
            trajectory.at(i).vx(), 
            trajectory.at(i).vy()
        );
        const double speed2 = vector_length(
            trajectory.at(i-1).vx(), 
            trajectory.at(i-1).vy()
        );
        const double average_speed = (speed1 + speed2)/2;

        const double delta_t = 1.05 * segment_length / average_speed;

        trajectory.at(i).t().nanoseconds(
            trajectory.at(i-1).t().nanoseconds() + uint64_t(1e9*delta_t)
        );
    }*/


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