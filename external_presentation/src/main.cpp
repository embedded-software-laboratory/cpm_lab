#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "Eight.hpp"
#include <dds/pub/ddspub.hpp>
#include <iostream>
#include <memory>
#include <stdlib.h>



using std::vector;

/*
 * Read this before you start:
 *     Vehicle Commands: http://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle+Commands
 *     Timer:            http://cpm.embedded.rwth-aachen.de/doc/pages/viewpage.action?pageId=2293786
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

    
    

    Eight eight;



    // These variabels track the reference state,
    // they are incremented as time passes.
    // int reference_trajectory_index = 0;
    uint64_t reference_trajectory_time = 0;

    // The "normal" eight trajectory is extended by one path connecting the two topmost points
    // and by one connecting the two lowermost points. If one of these paths is used the
    // direction in which the vehicle follows the 8-trajectory changes. Thus, all velocities
    // must change the sign which is done by this variable:
    // int trajectory_direction = 1;

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
        std::pair<TrajectoryPoint, uint64_t> p = eight.get_waypoint();
        TrajectoryPoint trajectory_point = p.first;
        trajectory_point.t().nanoseconds(reference_trajectory_time);

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
            std::cout << p.second <<std::endl;
            reference_trajectory_time += p.second;
            eight.move_forward();
            p = eight.get_waypoint();
            //reference_trajectory_index = (reference_trajectory_index + 1) % segment_duration.size();
        }

    });
}