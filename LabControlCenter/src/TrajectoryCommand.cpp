#include "TrajectoryCommand.hpp"

TrajectoryCommand::TrajectoryCommand()
{
    timer = cpm::Timer::create("LabControlCenter_TrajectoryCommand",40000000ull, 0, false,true);

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


void TrajectoryCommand::set_path(uint8_t vehicle_id, std::vector<Point> path, int n_loop)
{
    if(path.size() < 3) return;
    //uint64_t t_start = timer->get_time() + 1000000000ull;



    /********** Generate trajectory from given path ********/

    const int loop_start_index = find_path_loop_start_index(path);
    if(loop_start_index < 0 && n_loop > 0) return; // no closed loop found, path not possible

    // Generate unrolled path
    vector<TrajectoryPoint> trajectory;
    int path_index = 0;
    while(1)
    {
        if(path_index >= path.size()) // path end reached? -> loop around
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
    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        // TODO
    }

    // Set the timing of the trajectory nodes
    // TODO


    std::lock_guard<std::mutex> lock(_mutex);

    // TODO write trajectory to 'vehicle_trajectories'

}


void TrajectoryCommand::stop(uint8_t vehicle_id)
{
    std::lock_guard<std::mutex> lock(_mutex);

}


void TrajectoryCommand::stop_all()
{
    std::lock_guard<std::mutex> lock(_mutex);
}




void TrajectoryCommand::send_trajectory(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(_mutex);

}