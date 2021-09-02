// Set to true to get additional information about execution time in stdout
#define TIMED true

#include "VehicleTrajectoryPlanner.hpp"

/**
 * \file VehicleTrajectoryPlanner.cpp
 * \ingroup decentral_routing
 */

VehicleTrajectoryPlanner::VehicleTrajectoryPlanner(){}

VehicleCommandTrajectory VehicleTrajectoryPlanner::get_trajectory_command(uint64_t t_now)
{
    VehicleCommandTrajectory vehicleCommandTrajectory;
    vehicleCommandTrajectory.vehicle_id(trajectoryPlan->get_vehicle_id());
    vehicleCommandTrajectory.trajectory_points(
            rti::core::vector<TrajectoryPoint>(trajectory_point_buffer)
    );
    vehicleCommandTrajectory.header().create_stamp().nanoseconds(t_now); //You just need to set t_now here, as it was created at t_now
    vehicleCommandTrajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull); //Hardcoded value from the planner (t_start), should be correct (this value should correlate with the trajectory point that should be valid at t_now)

    return vehicleCommandTrajectory;
}

void VehicleTrajectoryPlanner::set_vehicle(std::unique_ptr<VehicleTrajectoryPlanningState> vehicle)
{
    assert(!started);
    trajectoryPlan = std::move(vehicle);
}

std::unique_ptr<VehicleCommandTrajectory> VehicleTrajectoryPlanner::plan(uint64_t t, uint64_t dt)
{
    isStopped = false;
    t_real_time = t;
    dt_nanos = dt;

    // Timesteps should come in a logical order
    assert(t_prev < t_real_time);

    // Catch up planningState if we missed a timestep
    while(t_real_time - t_prev > dt && t_prev !=0) {
        trajectoryPlan->apply_timestep(dt_nanos);
        t_prev += dt_nanos;
    }
    t_prev = t_real_time;

    started = true;
    // Messages_received gets reset every timestep
    all_received_messages.clear();

    // Read HlcCommunication messages and write them into our buffer
    read_previous_vehicles(); // Waits until we received the correct messages
    cpm::Logging::Instance().write(3,
            "%lu: Starting planning", t_real_time);

    // This loop runs until we get a stopFlag or all collisions are resolved
    bool has_collisions = true;
    bool received_collisions = true;
    do
    {
        has_collisions = !trajectoryPlan->avoid_collisions(other_vehicles_buffer);
        send_plan_to_hlcs(false, has_collisions);
        received_collisions = read_concurrent_vehicles();
        std::cout << "StopFlag says: " << stopFlag << std::endl;
    } while(
            !stopFlag && (received_collisions || has_collisions)
        );

    // If we still have collisions here, something went wrong
    if (received_collisions || has_collisions){
        cpm::Logging::Instance().write(1,
                "Found unavoidable collision");
        crashed = true;
        started = false; // end planning
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }

    if( !stopFlag ) {
        send_plan_to_hlcs();

        if(t_start == 0)
        {
            t_start = t_real_time;
        }
        else {
            clear_past_trajectory_point_buffer();
        }
        
        // Get new points from PlanningState
        std::vector<TrajectoryPoint> new_trajectory_points = trajectoryPlan->get_planned_trajectory(35, dt_nanos);
        for( auto& point : new_trajectory_points ) {
            point.t().nanoseconds(point.t().nanoseconds() + t_start);
        }

        // Append points to trajectory_point_buffer
        trajectory_point_buffer.insert(
               trajectory_point_buffer.end(),
               new_trajectory_points.begin(),
               new_trajectory_points.end()
               );

        // Limit size of trajectory buffer; delete oldest points first
        while( trajectory_point_buffer.size() > 50 ) {
            trajectory_point_buffer.erase(trajectory_point_buffer.begin());
        }
    }
    
    // Useful debugging tool if you suspect that trajectories aren't in sync between vehicles
    //std::cout << "Time " << t_real_time << std::endl;
    //debug_writeOutReceivedTrajectories();
    //trajectoryPlan->debug_writeOutOwnTrajectory();

    // Advance trajectoryPlanningState by 1 timestep
    trajectoryPlan->apply_timestep(dt_nanos);

    wait_for_ignored_vehicles();
    if ( !stopFlag ) {
        isStopped = true;
        no_trajectory_counter = 0;
        return std::unique_ptr<VehicleCommandTrajectory>(
                new VehicleCommandTrajectory(get_trajectory_command(t_real_time)));
    } else {
        cpm::Logging::Instance().write(2,
                "%lu: Not returning trajectory", t_real_time);
        no_trajectory_counter++;
        // If no trajectory was sent for 500ms, vehicles stop
        // Except for the first , like, 4 seconds, where it's normal
        if(no_trajectory_counter*dt_nanos > 500000000ull && (t_real_time - t_start > 4000000000ull)){
            cpm::Logging::Instance().write(1,
                    "Planner missed too many timesteps");
            crashed = true;
            started = false; // end planning
        }
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }
}

/*
 *
 */
void VehicleTrajectoryPlanner::read_previous_vehicles() {
    read_vehicles(coupling_graph.getPreviousVehicles(trajectoryPlan->get_vehicle_id()), true);
}

/*
 *
 */
bool VehicleTrajectoryPlanner::read_concurrent_vehicles() {
    return read_vehicles(coupling_graph.getConcurrentVehicles(trajectoryPlan->get_vehicle_id()), true, false); }

/*
 *
 */
void VehicleTrajectoryPlanner::wait_for_ignored_vehicles() {
    read_vehicles(coupling_graph.getIgnoredVehicles(trajectoryPlan->get_vehicle_id()));
}

/*
 * Reads LaneGraphTrajectories sent by other vehicles in this timestep
 * into other_vehicles_buffer.
 * Blocks until a message from all vehicles specified in the comm graph
 * is received or the stop() method is called.
 */
bool VehicleTrajectoryPlanner::read_vehicles(std::set<uint8_t> vehicle_ids, bool write_to_buffer, bool final_messages_only)
{
    assert(started);

#if TIMED
    auto start_time = std::chrono::steady_clock::now();
#endif

    // Loop until we receive a stopFlag OR
    // our messages_received contains all vehicles we're waiting for
    std::set<uint8_t> received_messages;
    int received_collisions = 0;
    while( !std::includes(received_messages.begin(), received_messages.end(),
                vehicle_ids.begin(), vehicle_ids.end()) // Becomes true, when we received msg from all vehicle_ids
            && !vehicle_ids.empty() // Stop immediately when vehicle_ids is empty
            && !stopFlag // Stop early, when we receive a stopFlag
            ) {
        auto samples = reader_hlcCommunication->take();
        for(HlcCommunication sample : samples) {
            uint64_t t_message = sample.header().create_stamp().nanoseconds();

            // Usually we only listen to messages with MessageType Final
            if ( final_messages_only && sample.type() != MessageType::Final ) {
                continue;
            }

            if ( t_message == t_real_time) {
                received_messages.insert(sample.vehicle_id());

                // Only process message if it's on the list of vehicles to read
                if ( vehicle_ids.find( sample.vehicle_id() )
                        == vehicle_ids.end()
                    ) {
                        std::cout << "Received message from "
                            << static_cast<uint32_t>(sample.vehicle_id())
                            << ", but cannot process it at this point."
                            << std::endl;
                        continue;
                }

                received_collisions += sample.has_collisions();

                // Only look at the lane graph positions if we need to write them to buffer
                if( !write_to_buffer) {
                    continue;
                }

                int prev_buffer_index = -1;
                int prev_edge_index = -1;
                int prev_edge_path_index = -1;

                // Clear out the buffer for this specific vehicle
                // Especially important for iterative planning vehicles
                other_vehicles_buffer[sample.vehicle_id()].clear();

                // Save all received positions that are not in the past already
                for ( LaneGraphPosition position : sample.lane_graph_positions() ) {

                    // Check TimeStamp of each Position to see where it fits into our buffer
                    // Casting to signed number because we might get negative values
                    long long t_eta = position.estimated_arrival_time().nanoseconds();
                    long long dt_speed_profile_nanos = trajectoryPlan->get_dt_speed_profile_nanos();
                    int index = 
                        ( t_eta - (long long) t_real_time)
                        / dt_speed_profile_nanos;

                    // We sometimes get unrealistic indices, which we don't want to use
                    if( index < -10 ) {
                        continue;
                    }

                    // Only write into buffer if index is positive,
                    // but still use negative indices for interpolation
                    if( index >= 0 ) {
                        other_vehicles_buffer[sample.vehicle_id()][index] =
                            std::make_pair(
                                position.edge_index(),
                                position.edge_path_index()
                            );
                    }

                    // If the prev_... vars are set, interpolate using them
                    // If not we cannot interpolate because there is just one point in the buffer
                    if( prev_edge_index >= 0 ) {
                        interpolate_other_vehicles_buffer(
                                sample.vehicle_id(),
                                prev_buffer_index, prev_edge_index, prev_edge_path_index,
                                index, position.edge_index(), position.edge_path_index()
                        );
                    }

                    // Remember these for interpolation purposes
                    prev_buffer_index = index;
                    prev_edge_index = position.edge_index();
                    prev_edge_path_index = position.edge_path_index();
                    
                }
            }
        }

        // Slow down loop a little
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

#if TIMED
    auto end_time = std::chrono::steady_clock::now();
    auto diff = end_time - start_time;
    std::cout << "TIMING: reading vehicles ";
    for(auto const &veh_id : vehicle_ids){
        std::cout << static_cast<uint32_t>(veh_id) << ", ";
    }
    std::cout
        << "took "
        << std::chrono::duration<double, std::milli>(diff).count()
        << " ms"
        << std::endl;
#endif

    return received_collisions;
}

/*
 * Stop planning of this timestep
 */
void VehicleTrajectoryPlanner::stop() {
    stopFlag = true; 
    // Block until planner is stopped
    while( !isStopped) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Stopped planning early at timestep " << t_real_time << std::endl;

    stopFlag = false;
    return;
}

/*
 * Publish the planned trajectory to DDS
 */
void VehicleTrajectoryPlanner::send_plan_to_hlcs(bool is_final, bool has_collisions) {

    HlcCommunication hlc_comms;
    trajectoryPlan->get_lane_graph_positions(&hlc_comms);
    // Add TimeStamp to each point
    for( unsigned int i=0; i<hlc_comms.lane_graph_positions().size(); i++ ) {
        // Calculate, which point in time this index corresponds to
        uint64_t eta = t_real_time + (trajectoryPlan->get_dt_speed_profile_nanos())*i;
        hlc_comms.lane_graph_positions()[i].estimated_arrival_time().nanoseconds(eta);
    }

    HlcCommunication msg;
    std::vector<LaneGraphPosition> shortened_trajectory;
    
    int current_edge_index = -1;
    LaneGraphPosition current_position;

    for( unsigned int i=0; i<hlc_comms.lane_graph_positions().size(); i++ ) {
        current_position = hlc_comms.lane_graph_positions()[i];

        // Only send each edge_index once
        if( current_position.edge_index() != current_edge_index ) {
            shortened_trajectory.push_back(current_position);
            current_edge_index = current_position.edge_index();
        }
        // Don't create oversize messages
        if( shortened_trajectory.size() >= msg_max_length ) {
            break;
        }
    }

    msg.lane_graph_positions(shortened_trajectory);
    
    if( is_final )
    {
        msg.type(MessageType::Final);
    }
    else
    {
        msg.type(MessageType::Iterative);
    }

    msg.has_collisions(has_collisions); 
    msg.vehicle_id(trajectoryPlan->get_vehicle_id());
    msg.header().create_stamp().nanoseconds(t_real_time);
    msg.header().valid_after_stamp().nanoseconds(t_real_time + 5000000000ull);
    this->writer_hlcCommunication->write(msg);
}

/*
 * Fills in points between two points on the Lane Graph in our other_vehicles_buffer.
 * This naively assumes a fixed speed, equidistant edge_paths-Points
 * and assumes that we are interpolating between two adjacent edges.
 */
void VehicleTrajectoryPlanner::interpolate_other_vehicles_buffer(
        uint8_t vehicle_id,
        int buffer_index_1, int edge_index_1, int edge_path_index_1,
        int buffer_index_2, int edge_index_2, int edge_path_index_2) {

    int delta_edge_path = edge_paths_per_edge - edge_path_index_1 + edge_path_index_2;
    int delta_index = buffer_index_2 - buffer_index_1;
    double step_size = delta_edge_path/delta_index;
    
    // Either start from one past buffer_index_1, or from 0 (buffer_index could be <0)
    for( int i = std::max(buffer_index_1+1, 0); i<buffer_index_2; i++ ) {
        int edge_path_to_insert = edge_path_index_1+int(step_size*(i-buffer_index_1+1));

        /* Interpolation starts on edge_index_1
         * and ends on edge_index_2.
         * Here we differentiate both cases.
         */
        int edge_to_insert;
        if( edge_path_to_insert < edge_paths_per_edge ) {
            edge_to_insert = edge_index_1;
        } else {
            edge_path_to_insert = edge_path_to_insert - edge_paths_per_edge;
            edge_to_insert = edge_index_2;
        }

        other_vehicles_buffer[vehicle_id][i] =
            std::make_pair(
                    edge_to_insert,
                    edge_path_to_insert
                    );
    }
}

/*
 * This deletes all points that are in the past from the trajectory point buffer
 * After that, we can just append the new points
 */
void VehicleTrajectoryPlanner::clear_past_trajectory_point_buffer() {
    // Delete all trajectory points that are in the future
    // These will be replaced by new points later
    std::vector<TrajectoryPoint>::iterator it_t_now = trajectory_point_buffer.end();
    for (std::vector<TrajectoryPoint>::iterator tp_it = trajectory_point_buffer.begin() + 1;
            tp_it != trajectory_point_buffer.end();
            ++tp_it) {
        if (tp_it->t().nanoseconds() > t_real_time
                && it_t_now == trajectory_point_buffer.end()) {
            it_t_now = tp_it-1;
        }
    }

    trajectory_point_buffer.erase(it_t_now, trajectory_point_buffer.end());
}

void VehicleTrajectoryPlanner::set_writer(
        std::unique_ptr< cpm::Writer<HlcCommunication> > writer){
    writer_hlcCommunication = std::move(writer);
}
void VehicleTrajectoryPlanner::set_reader(
        std::unique_ptr< cpm::ReaderAbstract<HlcCommunication> > reader){
    reader_hlcCommunication = std::move(reader);
}

/*
 * Prints contents of other_vehicles_buffer to stdout.
 */
void VehicleTrajectoryPlanner::debug_writeOutReceivedTrajectories() {
    for(auto vehicle : other_vehicles_buffer) {
        std::cout << "Vehicle " << static_cast<uint32_t>(vehicle.first) << std::endl;
        for(auto point : vehicle.second) {
            std::cout
                << point.first
                << ", " 
                << point.second.first
                << ", "
                << point.second.second
                << std::endl; 
        }
    }
}

/*
 * Debug method, that writes the current trajectory point buffer to stdout
 * and prints "WARNING", when there are inconsistencies.
 * Might print unnecessary DISTANCEWARNINGs at large middleware_periods
 */
void VehicleTrajectoryPlanner::debug_analyzeTrajectoryPointBuffer() {
    // Check quality of trajectory buffer
    std::cout << "trajectory_point_buffer:" << std::endl;
    std::cout << "Timestep: " << t_real_time << std::endl;
    auto prev_point = trajectory_point_buffer[0];
    std::cout
        << trajectory_point_buffer[0].t().nanoseconds()
        << ":\t"
        <<  trajectory_point_buffer[0].px()
        << ",\t"
        << trajectory_point_buffer[0].py()
        << std::endl;
    for( unsigned int i=1; i<trajectory_point_buffer.size(); i++ ) {
        // This int is currently signed, because it could be negative, but it shouldn't be
        double distance = sqrt(
                pow(
                    ((double)trajectory_point_buffer[i].px() - (double)prev_point.px()),
                    2
                )
                + pow(
                    ((double)trajectory_point_buffer[i].py() - (double)prev_point.py()),
                    2
                ));
	    int64_t time_diff = (int64_t) trajectory_point_buffer[i].t().nanoseconds() - (int64_t) prev_point.t().nanoseconds();
        std::cout
            << trajectory_point_buffer[i].t().nanoseconds()
            << ":\t"
            << trajectory_point_buffer[i].px()
            << ",\t"
            << trajectory_point_buffer[i].py();
        if( distance > 0.6) {
            std::cout << " DISTANCEWARNING " << distance;
        }
	    if( time_diff != (int64_t) dt_nanos ) {
            std::cout << " TIMEWARNING " << time_diff;
	    }
        std::cout << std::endl;
	    prev_point = trajectory_point_buffer[i];
    }

}
