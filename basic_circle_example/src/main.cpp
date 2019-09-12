
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


    const uint64_t dt_nanos = 400000000ull;

    // Writer for sending trajectory commands
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );

    int num_trajectory_points = 10;
    uint64_t t_start = 0;
    auto timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, enable_simulated_time);


    
    vector<double> circle_px = vector<double>{ 1, 0,-1, 0};
    vector<double> circle_py = vector<double>{ 0, 1, 0,-1};
    vector<double> circle_vx = vector<double>{ 0,-1, 0, 1};
    vector<double> circle_vy = vector<double>{ 1, 0,-1, 0};

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

    // duration ~= PI*d/4
    // vector<double> t_section = vector<double>{1.55, 1.55, 1.55, 1.55};
    // vector<double> t_section_added = t_section;
    // double t_cycle = 0;

    // for (uint i = 1; i < t_section_added.size(); ++i)
    // {
    //     t_section_added[i] += t_section_added[i-1];
    // }
    // for (double &t : t_section)
    // {
    //     t_cycle += t;
    // }
    vector<uint64_t> t_section = vector<uint64_t>{1550000000ull, 1550000000ull, 1550000000ull, 1550000000ull};
    vector<uint64_t> t_section_added = t_section;

    for (uint i = 1; i < t_section_added.size(); ++i)
    {
        t_section_added[i] += t_section_added[i-1];
    }
    uint64_t t_cycle = t_section_added.back();

    timer->start([&](uint64_t t_now)
    {
        // time used for trajectory generation
        if (t_start == 0) t_start = t_now;

        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_id);
        vector<TrajectoryPoint> trajectory_points;

        for (int i = 0; i < num_trajectory_points; ++i)
        {
            TrajectoryPoint trajectory_point;
            uint64_t time_point_nanos = t_now - t_start + dt_nanos*i;

            // find 'section' in which vehicle currently moves
            uint64_t t_rel = time_point_nanos % t_cycle;
            auto it_sec = std::upper_bound(t_section_added.begin(), t_section_added.end(), t_rel);
            uint section_idx = it_sec - t_section_added.begin();

            // timestamp for next trajectory point
            uint64_t t_next_traj_pt_nanos = 
                time_point_nanos
                + t_start
                + t_section_added[section_idx]
                - t_rel;

            trajectory_point.t().nanoseconds(t_next_traj_pt_nanos);
            trajectory_point.px(circle_px[section_idx]);
            trajectory_point.py(circle_py[section_idx]);
            trajectory_point.vx(circle_vx[section_idx]);
            trajectory_point.vy(circle_vy[section_idx]);
            trajectory_points.push_back(trajectory_point);
            std::cout << trajectory_point << std::endl;
        }
        vehicle_command_trajectory.trajectory_points(trajectory_points);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
        std::cout << std::endl;
    });
}