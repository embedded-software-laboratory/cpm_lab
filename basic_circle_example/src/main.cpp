
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>
#include <memory>

using std::vector;

int main(int argc, char *argv[])
{
    cpm::init(argc, argv);
    std::string node_id = "basic_circle_example";
    cpm::Logging::Instance().set_id(node_id);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    int vehicle_id_int = cpm::cmd_parameter_int("vehicle_id", {4}, argc, argv);
    uint8_t vehicle_id;
    assert(vehicle_id_int > 0);
    assert(vehicle_id_int < 255);
    vehicle_id = vehicle_id_int;



    // Writer for sending trajectory commands
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );

    
    vector<double> circle_px = vector<double>{ 1, 0,-1, 0};
    vector<double> circle_py = vector<double>{ 0, 1, 0,-1};
    vector<double> circle_vx = vector<double>{ 0,-1, 0, 1};
    vector<double> circle_vy = vector<double>{ 1, 0,-1, 0};
    // duration to follow a trajectory segment:
    // ~= 2PIr/4 / v    with (v=1m/s), (r=1m)
    vector<uint64_t> t_section = vector<uint64_t>{1550000000ull, 1550000000ull, 1550000000ull, 1550000000ull};
    vector<uint64_t> t_section_added = t_section;

    double map_center_x = 2.25;
    double map_center_y = 2.0;
    for (double &px : circle_px)
    {
        px += map_center_x;
    }
    for (double &py : circle_py)
    {
        py += map_center_y;
    }

    int reference_trajectory_index = 0;
    uint64_t reference_trajectory_time = 0;


    const uint64_t dt_nanos = 400000000ull;
    auto timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now)
    {
        // time used for trajectory generation
        if (reference_trajectory_time == 0) reference_trajectory_time = t_now;

        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_id);
        vector<TrajectoryPoint> trajectory_points;


        TrajectoryPoint trajectory_point;
        trajectory_point.t().nanoseconds(reference_trajectory_time);
        trajectory_point.px(circle_px[reference_trajectory_index]);
        trajectory_point.py(circle_py[reference_trajectory_index]);
        trajectory_point.vx(circle_vx[reference_trajectory_index]);
        trajectory_point.vy(circle_vy[reference_trajectory_index]);
        trajectory_points.push_back(trajectory_point);


        while(reference_trajectory_time < t_now + 2000000000ull)
        {
            reference_trajectory_time += t_section[reference_trajectory_index];
            reference_trajectory_index = (reference_trajectory_index + 1) % t_section.size();
        }

        vehicle_command_trajectory.trajectory_points(trajectory_points);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
    });
}