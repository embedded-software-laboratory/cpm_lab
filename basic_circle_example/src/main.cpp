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

/*
 * Read this before you start:
 *     Vehicle Commands: http://cpm-lab.embedded.rwth-aachen.de:8090/display/CLD/Vehicle+Commands
 *     Timer:            http://cpm-lab.embedded.rwth-aachen.de:8090/pages/viewpage.action?pageId=2293786
 */



int main(int argc, char *argv[])
{
    //Prepare for Logging
    const std::string node_id = "basic_circle_example";
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id(node_id);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    //set vehicle ID: ID must be the same in code and the LCC!
    int vehicle_id_int = cpm::cmd_parameter_int("vehicle_id", 4, argc, argv);
    uint8_t vehicle_id;
    assert(vehicle_id_int > 0);
    assert(vehicle_id_int < 255);
    vehicle_id = vehicle_id_int;


    // Writer for sending trajectory commands, Writer writes the trajectory commands in the DDS "Cloud" so other programs can access them.
    //For more information see our documentation about RTI DDS
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<VehicleCommandTrajectory>("vehicleCommandTrajectory")
    );

    // Circle trajectory data
    //In this section the points on the x and y axis (independently from the map!) are set. 
    //They are relative to the defined center point defined below as map_center_x and map_center_y
    vector<double> trajectory_px        = vector<double>{            1,             0,            -1,             0};
    vector<double> trajectory_py        = vector<double>{            0,             1,             0,            -1};
    //These vecotrs define the speed in x and y direction. Together the define the starting direction from the current trajectory point,
    // for example: vx = 1 and vy = 1 will lead to a positive diagonal starting vector from the starting point. For more informations see
    //our documentation website
    vector<double> trajectory_vx        = vector<double>{            0,            -1,             0,             1};
    vector<double> trajectory_vy        = vector<double>{            1,             0,            -1,             0};

    //This vector defines the duration for one way segment. So, in a way it defines the acutal speed of the vehicle in one segment. 
    //In this vector we see the time for v = 1 m/s (as every vx and vy is either 0 or 1) and a way length of pi/2 (a quater circle ;))
    //Find more hints on our documentation website
    vector<uint64_t> segment_duration = vector<uint64_t>{1550000000ull, 1550000000ull, 1550000000ull, 1550000000ull};

    /*
    // Figure eight trajectory data
    vector<double> trajectory_px        = vector<double>{           -1,             0,             1,             0};
    vector<double> trajectory_py        = vector<double>{            0,             0,             0,             0};
    //In this figure eight the circles of the eight are not perfectly round but a little streched to show you the impact of the vector (Vx, vy)
    //at the starting point. 
    //Note that also the duration changes accordingly.
    vector<double> trajectory_vx        = vector<double>{            0,          0.14,             0,         -0.14};
    vector<double> trajectory_vy        = vector<double>{          1.3,         -1.27,           1.3,         -1.27};
    vector<uint64_t> segment_duration = vector<uint64_t>{1700000000ull, 1700000000ull, 1700000000ull, 1700000000ull};
    */

    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());

    //Definition of the center point for the circle and the figure eight
    const double map_center_x = 2.25;
    const double map_center_y = 2.0;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }


    // These variabels track the reference state,
    // they are incremented as time passes.
    int reference_trajectory_index = 0;
    uint64_t reference_trajectory_time = 0;


    // The code inside the cpm::Timer is executed every 400 milliseconds.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    const uint64_t dt_nanos = 400000000ull; // 400 milliseconds == 400000000 nanoseconds
    auto timer = cpm::Timer::create(node_id, dt_nanos, 0, false, true, enable_simulated_time);
    timer->start([&](uint64_t t_now)
    {
        // Initial time used for trajectory generation
        if (reference_trajectory_time == 0) reference_trajectory_time = t_now;


        // Send the current trajectory point to the vehicle
        TrajectoryPoint trajectory_point;
        trajectory_point.t().nanoseconds(reference_trajectory_time);
        trajectory_point.px(trajectory_px[reference_trajectory_index]);
        trajectory_point.py(trajectory_py[reference_trajectory_index]);
        trajectory_point.vx(trajectory_vx[reference_trajectory_index]);
        trajectory_point.vy(trajectory_vy[reference_trajectory_index]);
        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_id);
        vehicle_command_trajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(1, trajectory_point));
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

        // Advance the reference state to T+2sec.
        // The reference state must be in the future,
        // to allow some time for the vehicle to receive
        // the message and anticipate the next turn.
        while(reference_trajectory_time < t_now + 2000000000ull)
        {
            reference_trajectory_time += segment_duration[reference_trajectory_index];
            reference_trajectory_index = (reference_trajectory_index + 1) % segment_duration.size();
        }

    });
}