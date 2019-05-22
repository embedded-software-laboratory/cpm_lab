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

int find_path_loop_start_index(std::vector<Point> path)
{
    if(path.size() < 3) return -1;

    // position and direction of last node
    const double x = path.at(path.size()-1).x;
    const double y = path.at(path.size()-1).y;
    const double dx = path.at(path.size()-1).x - path.at(path.size()-2).x;
    const double dy = path.at(path.size()-1).y - path.at(path.size()-2).y;


    // find a suitable node in the path to close the loop
    int loop_start_index = -1;
    double dist_sq_min = 1e300;

    for (size_t i = 0; i < path.size()-2; ++i)
    {
        auto point = path.at(i);
        double dist_sq = (point.x-x)*(point.x-x) + (point.y-y)*(point.y-y);
        double direction = point.x * dx + point.y * dy;

        if(direction > 0 && dist_sq > 0.04 && dist_sq < dist_sq_min)
        {
            loop_start_index = i;
            dist_sq_min = dist_sq;
        }
    }
    return loop_start_index;    
}

inline double vector_length(double x, double y)
{
    return sqrt(x*x+y*y);
}


void TrajectoryCommand::set_path(uint8_t vehicle_id, std::vector<Point> path, int n_loop)
{
    if(path.size() < 3) return;



    /********** Generate trajectory from given path ********/

    const int loop_start_index = find_path_loop_start_index(path);
    if(loop_start_index < 0 && n_loop > 0) return; // no closed loop found, path not possible

    // Generate unrolled path
    vector<TrajectoryPoint> trajectory;
    int path_index = 0;
    while(1)
    {
        if(path_index >= int(path.size())) // path end reached? -> loop around
        {
            path_index = loop_start_index;
            n_loop--;
            if(n_loop < 0) // all loops done? -> finish
            {
                break;
            }
        }

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
    uint64_t t_start = timer->get_time() + 1000000000ull;
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
            if(t_now + 500000000ull < trajectory.at(i).t().nanoseconds())
            {
                auto trajectoryPoint = trajectory.at(i);
                VehicleCommandTrajectory command;
                command.vehicle_id(vehicle_id);
                command.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectoryPoint));
                cpm::stamp_message(command, t_now, 20000000ull);
                writer_vehicleCommandTrajectory.write(command);
                break;
            }

            // TODO delete trajectory if it is in the past
        }

    }
}