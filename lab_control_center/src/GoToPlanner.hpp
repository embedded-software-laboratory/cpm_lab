#pragma once

#include <memory> // shared_ptr
#include <thread>
#include <vector>

#include "Pose2D.hpp"
#include "TimeSeriesAggregator.hpp"
#include "TrajectoryCommand.hpp"

/**
 * \class GoToPlanner
 * \brief Class to control vehicles to poses 
 * \ingroup lcc
 */
class GoToPlanner {
public:
    GoToPlanner(
        std::function<std::vector<Pose2D>()> get_goal_poses
        ,std::function<VehicleData()> get_vehicle_data
        ,std::shared_ptr<TrajectoryCommand> trajectory_command
        ,std::string absolute_executable_path
    );

    void go_to_start_poses();
    void go_to_poses(std::vector<Pose2D> goal_poses);

private:
    std::function<std::vector<Pose2D>()> get_goal_poses;
    std::function<VehicleData()> get_vehicle_data;
    std::shared_ptr<TrajectoryCommand> trajectory_command;
    std::thread planner_thread;
    std::string matlab_functions_path;
};