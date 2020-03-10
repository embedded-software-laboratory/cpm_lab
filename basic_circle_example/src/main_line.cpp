#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "cpm/get_time_ns.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>
#include <memory>

using std::vector;

/*
 * Read this before you start:
 *     Vehicle Commands: http://cpm-lab.embedded.rwth-aachen.de:8090/display/CLD/Vehicle+Commands
 *     Timer:            http://cpm-lab.embedded.rwth-aachen.de:8090/pages/viewpage.action?pageId=2293786
 */



int main(int argc, char *argv[])
{
    const std::string node_id = "basic_circle_example";
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id(node_id);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    int vehicle_id_int = cpm::cmd_parameter_int("vehicle_id", 4, argc, argv);
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

    // Circle trajectory data
    // vector<double> trajectory_px        = vector<double>{            1,             0,            -1,             0};
    // vector<double> trajectory_py        = vector<double>{            0,             1,             0,            -1};
    // vector<double> trajectory_vx        = vector<double>{            0,            -0.5,             0,             0.5};
    // vector<double> trajectory_vy        = vector<double>{            0.5,             0,            -0.5,             0};
    // vector<uint64_t> segment_duration = vector<uint64_t>{2*1550000000ull, 2*1550000000ull, 2*1550000000ull, 2*1550000000ull};

    vector<double> trajectory_px        = vector<double>{-0.5, 0,  0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    vector<double> trajectory_py        = vector<double>{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    vector<double> trajectory_vx        = vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    vector<double> trajectory_vy        = vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    vector<uint64_t> segment_duration = vector<uint64_t>{1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull, 1000000000ull};

    /*
    // Figure eight trajectory data
    vector<double> trajectory_px        = vector<double>{           -1,             0,             1,             0};
    vector<double> trajectory_py        = vector<double>{            0,             0,             0,             0};
    vector<double> trajectory_vx        = vector<double>{            0,          0.14,             0,         -0.14};
    vector<double> trajectory_vy        = vector<double>{          1.3,         -1.27,           1.3,         -1.27};
    vector<uint64_t> segment_duration = vector<uint64_t>{1700000000ull, 1700000000ull, 1700000000ull, 1700000000ull};
    */

    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());

    const double map_center_x = 0.25;
    const double map_center_y = 2.0;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }

    auto t_now = cpm::get_time_ns();
    uint64_t reference_trajectory_time = t_now + 10000000000ull;
    size_t reference_trajectory_index = 0;

    //Vector of trajectory points (send whole circle for interpolation, always with the current point as second point to interpolate to)
    vector<TrajectoryPoint> trajectory_points;
    for (size_t i = 0; i < segment_duration.size(); ++i)
    {
        size_t trajectory_index = (reference_trajectory_index + i) % segment_duration.size();
        uint64_t trajectory_time = reference_trajectory_time + (i - 1) * segment_duration[reference_trajectory_index];

        TrajectoryPoint trajectory_point;
        trajectory_point.px(trajectory_px[trajectory_index]);
        trajectory_point.py(trajectory_py[trajectory_index]);
        trajectory_point.vx(trajectory_vx[trajectory_index]);
        trajectory_point.vy(trajectory_vy[trajectory_index]);
        trajectory_point.t().nanoseconds(trajectory_time); //This needs to be improved in case the durations are different

        trajectory_points.push_back(trajectory_point);
    }

    // Send the current trajectory 
    rti::core::vector<TrajectoryPoint> rti_trajectory_points(trajectory_points);
    VehicleCommandTrajectory vehicle_command_trajectory;
    vehicle_command_trajectory.vehicle_id(1);
    vehicle_command_trajectory.trajectory_points(rti_trajectory_points);
    vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
    vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(reference_trajectory_time);
    
    for (int i = 0; i < 10; ++i)
    {
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
        sleep(1);
    }

    std::cout << "Sent: " << vehicle_command_trajectory << std::endl;

    sleep(10);
}