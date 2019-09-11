
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

    double map_center_x = 2.25;
    double map_center_y = 2.0;
    int num_trajectory_points = 5;
    uint64_t t_start = 0;
    auto timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, enable_simulated_time);


    
    vector<double> circle_px = vector<double>{ 1, 0,-1, 0};
    vector<double> circle_py = vector<double>{ 0, 1, 0,-1};
    vector<double> circle_vx = vector<double>{ 0,-1, 0, 1};
    vector<double> circle_vy = vector<double>{ 1, 0,-1, 0};

    for (double &px : circle_px)
    {
        px += map_center_x;
    }
    for (double &py : circle_py)
    {
        py += map_center_y;
    }

    // duration ~= PI*d/4
    vector<double> t_section = vector<double>{ 0, 1.55, 3.1, 4.65};
    double t_cycle = t_section.back();

    timer->start([&](uint64_t t_now)
    {
        // time used for trajectory generation
        if (t_start == 0)
        {
            t_start = t_now;
        }
        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_id);
        vector<TrajectoryPoint> trajectory_points;

        for (int i = 0; i < num_trajectory_points; ++i)
        {
            TrajectoryPoint trajectory_point;
            uint64_t time_point = t_now - t_start + dt_nanos*i;

            // was muss ich addieren
            // 

            // find 'section' in which vehicle currently moves
            double t_rel = fmod(time_point*1e-9, t_cycle);
            auto it_sec = std::upper_bound(t_section.begin(), t_section.end(), t_rel);
            uint section_idx = it_sec - t_section.begin();


            double t_at_next_point = t_now + t_sec - t_rel + i*dt_nanos;
            if (section_idx == 5)
            {
                section_idx = 0;
            }
            trajectory_point.px(circle_px[section_idx]);
            trajectory_point.py(circle_py[section_idx]);
            trajectory_point.vx(circle_vx[section_idx]);
            trajectory_point.vy(circle_vy[section_idx]);
            trajectory_points.push_back(trajectory_point);
        }
        vehicle_command_trajectory.trajectory_points(trajectory_points);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
    });
}