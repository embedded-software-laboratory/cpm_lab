#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/HLCCommunicator.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>
#include <memory>

using std::vector;

//Description for bash files
/**
 * \defgroup two_vehicles_drive_files Additional Files
 * \ingroup two_vehicles_drive
 */

/**
 * \page two_vehicles_drive_files_page Additional Files for Two Vehicles Drive
 * \subpage t_v_d_build <br>
 * \subpage t_v_d_run <br>
 * \ingroup two_vehicles_drive_files
*/

/**
 * \page t_v_d_build build.bash
 * \brief Build script for two_vehicles_drive
 */

/**
 * \page t_v_d_run run.bash
 * \brief Run script for two_vehicles_drive
 */

/**
 * \brief Main function of the two_vehicles_drive scenario
 * This tutorial is also described at https://cpm.embedded.rwth-aachen.de/doc/pages/viewpage.action?spaceKey=CLD&title=Two+Vehicles+Drive
 * \param argc Command line argument
 * \param argv Command line argument
 * \ingroup two_vehicles_drive
 */
int main(int argc, char *argv[])
{
    //Initialize cpm library
    const std::string node_id = "two_vehicles_drive";
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id(node_id);
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);
    std::vector<uint8_t> vehicle_ids;
    for (auto i : vehicle_ids_int)
    {
        assert(i > 0);
        assert(i < 255);
        vehicle_ids.push_back(i);
    }
    assert(vehicle_ids.size() > 0);

    //Give user time to drag-and-drop the vehicle to the right position in simulation
    //(The vehicle does not consider 'mixed' trajectories as before, when trajectories were sent e.g. by both the LCC and the program)
    sleep(10);

    HLCCommunicator hlc_communicator(
            vehicle_ids
    );

    // Writer for sending trajectory commands, Writer writes the trajectory commands in the DDS "Cloud" so other programs can access them.
    // Instead of creating a new participant, we can just use the one created by the HLCCommunicator
    //For more information see our documentation about RTI DDS
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
            hlc_communicator.getLocalParticipant()->get_participant(),
            "vehicleCommandTrajectory");

    // Circle trajectory data
    //In this section the points on the x and y axis (independently from the map!) are set.
    //They are relative to the defined center point defined below as map_center_x and map_center_y
    vector<double> trajectory_px = vector<double>{1, 0, -1, 0};
    vector<double> trajectory_py = vector<double>{0, 1, 0, -1};
    //These vecotrs define the speed in x and y direction. Together the define the starting direction from the current trajectory point,
    // for example: vx = 1 and vy = 1 will lead to a positive diagonal starting vector from the starting point. For more informations see
    //our documentation website
    vector<double> trajectory_vx = vector<double>{0, -1, 0, 1};
    vector<double> trajectory_vy = vector<double>{1, 0, -1, 0};
    vector<uint64_t> segment_duration = vector<uint64_t>{1570800000ull, 1570800000ull, 1570800000ull, 1570800000ull};

    // Circle trajectory data for the second vehicle
    vector<double> trajectory_px2 = vector<double>{1, 0, -1, 0};
    vector<double> trajectory_py2 = vector<double>{0, -1, 0, 1};
    //These vecotrs define the speed in x and y direction. 
    vector<double> trajectory_vx2 = vector<double>{-0.01, -1, 0.01, 1};
    vector<double> trajectory_vy2 = vector<double>{-1, 0.01, 1, -0.01};


    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());

    //Definition of the center point for the circle and the figure eight
    const double map_center_x = 3.1;
    const double map_center_y = 1.1;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }

    const double map_center_x2 = 1.25;
    const double map_center_y2 = 2.9;

    for (double &px2 : trajectory_px2)
    {
        px2 += map_center_x2;
    }
    for (double &py2 : trajectory_py2)
    {
        py2 += map_center_y2;
    }

    // These variabels track the reference state,
    // they are incremented as time passes.
    int reference_trajectory_index = 0;
    uint64_t reference_trajectory_time = 0;
    uint64_t t_now = 0;

    // The code inside the onEachTimestep method is executed each timestep.
    // Here we assume that we send trajectories every 200ms
    // This means we need to manually set the middleware_period_ms parameter in the LCC to 200ms.
    // Commands must be sent to the vehicle regularly, more than 2x per second.
    // Otherwise it is assumed that the connection is lost and the vehicle stops.
    const uint64_t dt_nanos = 200000000ull; // 200 milliseconds == 200000000 nanoseconds

    // This code will get executed only once at the beginning of the first timestep
    hlc_communicator.onFirstTimestep([&](VehicleStateList vehicle_state_list)
    {
        // Initial time used for trajectory generation
        reference_trajectory_time = vehicle_state_list.t_now() + 1000000000ull;
    });

    hlc_communicator.onEachTimestep([&](VehicleStateList vehicle_state_list)
    {
        // Check if middleware_period_ms was set correctly, as described above
        // If not, write a message to log
        if( vehicle_state_list.period_ms()*1000000ull != dt_nanos ){
            cpm::Logging::Instance().write(1,
                    "Please set middleware_period_ms to 200ms");
            return;
        }

        t_now = vehicle_state_list.t_now();

        vector<TrajectoryPoint> trajectory_points;
        vector<TrajectoryPoint> trajectory_points2;
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

            TrajectoryPoint trajectory_point2;
            trajectory_point2.px(trajectory_px2[trajectory_index]);
            trajectory_point2.py(trajectory_py2[trajectory_index]);
            trajectory_point2.vx(trajectory_vx2[trajectory_index]);
            trajectory_point2.vy(trajectory_vy2[trajectory_index]);
            trajectory_point2.t().nanoseconds(trajectory_time); //This needs to be improved in case the durations are different
            trajectory_points2.push_back(trajectory_point2);
        }

        

       // Send the current trajectory, each vehicle gets its own trajectory
        rti::core::vector<TrajectoryPoint> rti_trajectory_points(trajectory_points);
        rti::core::vector<TrajectoryPoint> rti_trajectory_points2(trajectory_points2);
        VehicleCommandTrajectory vehicle_command_trajectory;
        vehicle_command_trajectory.vehicle_id(vehicle_ids.at(0));
        vehicle_command_trajectory.trajectory_points(rti_trajectory_points);
        vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
        vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
        vehicle_command_trajectory.vehicle_id(vehicle_ids.at(1));
        vehicle_command_trajectory.trajectory_points(rti_trajectory_points2);
        vehicle_command_trajectory.header().create_stamp().nanoseconds(t_now);
        vehicle_command_trajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory); 



        // Advance the reference state to T+1sec.
        // The reference state must be in the future,
        // to allow some time for the vehicle to receive
        // the message and anticipate the next turn.
        // We repeat the current message until the segment_duration is lower than the passed time
        while(reference_trajectory_time + segment_duration[reference_trajectory_index] < t_now + 1000000000ull)
        {
        reference_trajectory_time += segment_duration[reference_trajectory_index];
        reference_trajectory_index = (reference_trajectory_index + 1) % segment_duration.size();
        }
    });

    hlc_communicator.start();
}
