#pragma once
#include <stdint.h>
#include "defaults.hpp"
#include "Point.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/Timer.hpp"
#include "cpm/stamp_message.hpp"

class TrajectoryCommand
{
    std::mutex _mutex;
    map<uint8_t, vector<TrajectoryPoint>> vehicle_trajectories;
    std::shared_ptr<cpm::Timer> timer;

    //shared_ptr<dds::pub::DataWriter<VehicleCommandTrajectory>> writer_vehicleCommandTrajectory;

    void send_trajectory(uint64_t t_now);

public:
    TrajectoryCommand();
    ~TrajectoryCommand();
    void set_path(uint8_t vehicle_id, std::vector<Point> path, int n_loop);
    void stop(uint8_t vehicle_id);
    void stop_all();
    
};