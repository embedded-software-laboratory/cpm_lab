#pragma once
#include <stdint.h>
#include "defaults.hpp"
#include "Point.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/TimerFD.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/get_topic.hpp"
#include <dds/pub/ddspub.hpp>

class TrajectoryCommand
{
    std::mutex _mutex;
    map<uint8_t, vector<TrajectoryPoint>> vehicle_trajectories;
    std::shared_ptr<cpm::TimerFD> timer;

    dds::topic::Topic<VehicleCommandTrajectory> topic_vehicleCommandTrajectory;
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory;


    void send_trajectory(uint64_t t_now);

public:
    TrajectoryCommand();
    ~TrajectoryCommand();
    void set_path(uint8_t vehicle_id, std::vector<Point> path, int n_loop);
    void stop(uint8_t vehicle_id);
    void stop_all();
    
};