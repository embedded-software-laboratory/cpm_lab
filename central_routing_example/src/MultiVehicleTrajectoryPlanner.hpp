#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include "VehicleCommandTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp"
using std::vector;


class MultiVehicleTrajectoryPlanner
{
    std::map<uint8_t, std::shared_ptr<VehicleTrajectoryPlanningState> > trajectoryPlans;
    bool started = false;
    uint64_t t_start = 0;
    uint64_t t_real_time = 0;
    std::mutex mutex;
    std::thread planning_thread;
    const uint64_t dt_nanos;
    std::map<uint8_t, std::vector<TrajectoryPoint> > trajectory_point_buffer;

public:

    MultiVehicleTrajectoryPlanner(uint64_t dt_nanos);
    std::vector<VehicleCommandTrajectory> get_trajectory_commands();
    void set_real_time(uint64_t t);
    bool is_started() {return started;}
    void add_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle);
    void start();

};
