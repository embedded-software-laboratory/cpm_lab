// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

// Set to true to get additional information about execution time in stdout
#define TIMED false

#include "VehicleTrajectoryPlanner.hpp"


VehicleTrajectoryPlanner::VehicleTrajectoryPlanner(){}

VehicleTrajectoryPlanner::~VehicleTrajectoryPlanner(){
    if ( planning_thread.joinable() ) planning_thread.join();
}

VehicleCommandTrajectory VehicleTrajectoryPlanner::get_trajectory_command(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(mutex);

    VehicleCommandTrajectory vehicleCommandTrajectory;
    vehicleCommandTrajectory.vehicle_id(trajectoryPlan->get_vehicle_id());
    vehicleCommandTrajectory.trajectory_points(
            rti::core::vector<TrajectoryPoint>(trajectory_point_buffer)
    );
    vehicleCommandTrajectory.header().create_stamp().nanoseconds(t_now); //You just need to set t_now here, as it was created at t_now
    vehicleCommandTrajectory.header().valid_after_stamp().nanoseconds(t_now + 1000000000ull); //Hardcoded value from the planner (t_start), should be correct (this value should correlate with the trajectory point that should be valid at t_now)

    return vehicleCommandTrajectory;
}

void VehicleTrajectoryPlanner::set_real_time(uint64_t t)
{
    std::lock_guard<std::mutex> lock(mutex);
    t_real_time = t;
}


void VehicleTrajectoryPlanner::set_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle)
{
    assert(!started);
    trajectoryPlan = vehicle;
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
    messages_received.clear();

    // Read LaneGraphTrajectory messages and write them into our buffer
    this->read_other_vehicles(); // Waits until we received the correct messages
    cpm::Logging::Instance().write(3,
            "%lu: Starting planning", t_real_time);

    //// Check if we should stop and return early if we do
    //if( stopFlag ) {
    //    std::cout << "Aborting before buffer" << std::endl;
    //    isStopped = true;
    //    return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    //}

    // Priority based collision avoidance: Every vehicle avoids 
    // the 'previous' vehicles, in this example those with a smaller ID.
    bool is_collision_avoidable = false;
    is_collision_avoidable = trajectoryPlan->avoid_collisions(other_vehicles_buffer);

    if (!is_collision_avoidable){
        cpm::Logging::Instance().write(1,
                "Found unavoidable collision");
        crashed = true;
        started = false; // end planning
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }


    if(t_start == 0)
    {
        t_start = t_real_time;// + 2000000000ull;
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

    //debug_analyzeTrajectoryPointBuffer();

    // Get our current trajectory
    LaneGraphTrajectory lane_graph_trajectory;
    trajectoryPlan->get_lane_graph_positions(
            &lane_graph_trajectory
    );

    // Advance trajectoryPlanningState by 1 timestep
    trajectoryPlan->apply_timestep(dt_nanos);

    //if(stopFlag) {
    //    isStopped = true;
    //    return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    //}

    //cpm::Logging::Instance().write(1,
    //        "%lu: Finished planning", t_real_time);
    //// Publish our planned trajectory with other vehicles
    //cpm::Logging::Instance().write(1,
    //        "Sending traj, vehicle %i",
    //        trajectoryPlan->get_vehicle_id()
    //        );
    write_trajectory(lane_graph_trajectory);

    
    // Useful debugging tool if you suspect that trajectories aren't in sync between vehicles
    //std::cout << "Time " << t_real_time << std::endl;
    //debug_writeOutReceivedTrajectories();
    //trajectoryPlan->debug_writeOutOwnTrajectory();
    

    if (wait_for_other_vehicles()) {
        isStopped = true;
        no_trajectory_counter = 0;
        return std::unique_ptr<VehicleCommandTrajectory>(new VehicleCommandTrajectory(get_trajectory_command(t_real_time)));
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
 * Reads LaneGraphTrajectories sent by other vehicles in this timestep
 * into other_vehicles_buffer.
 * Blocks until a message from all vehicles specified in the comm graph
 * is received or the stop() method is called.
 */
void VehicleTrajectoryPlanner::read_other_vehicles()
{
    assert(started);

#if TIMED
    auto start_time = std::chrono::steady_clock::now();
#endif

    // Clear buffer from previous timestep
    other_vehicles_buffer.clear();

    // Checklist to check, which messages we need to wait for
    std::set<uint8_t> prev_vehicles_list = coupling_graph.getPreviousVehicles(trajectoryPlan->get_vehicle_id());

    bool still_waiting = true;
    while( still_waiting && !stopFlag) {
        auto samples = reader_laneGraphTrajectory->take();
        for(auto sample : samples) {
            LaneGraphTrajectory data = sample.data();
            uint64_t t_message = data.header().create_stamp().nanoseconds();

            if (sample.info().valid() && t_message == t_real_time) {
                messages_received.insert(data.vehicle_id());

                // Only process message if it's from a previous vehicle
                if ( prev_vehicles_list.find( data.vehicle_id() )
                        == prev_vehicles_list.end()
                    ) {
                        continue;
                }

                int prev_buffer_index = -1;
                int prev_edge_index = -1;
                int prev_edge_path_index = -1;

                // Save all received positions that are not in the past already
                for ( LaneGraphPosition position : data.lane_graph_positions() ) {


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
                        other_vehicles_buffer[data.vehicle_id()][index] =
                            std::make_pair(
                                position.edge_index(),
                                position.edge_path_index()
                            );
                    }

                    // If the prev_... vars are set, interpolate using them
                    // If not we cannot interpolate because there is just one point in the buffer
                    if( prev_edge_index >= 0 ) {
                        interpolate_other_vehicles_buffer(
                                data.vehicle_id(),
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

        // Check if we finished our checklist; if yes: exit loop
        still_waiting = !std::includes(messages_received.begin(), messages_received.end(),
                prev_vehicles_list.begin(), prev_vehicles_list.end());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

#if TIMED
    auto end_time = std::chrono::steady_clock::now();
    auto diff = end_time - start_time;
    std::cout << "TIMING: read_other_vehicles took " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
#endif
}

bool VehicleTrajectoryPlanner::wait_for_other_vehicles() {
#if TIMED
    auto start_time = std::chrono::steady_clock::now();
#endif

    // If we have a stopFlag, no need to wait
    bool stop_waiting = stopFlag;
    bool success_status = false;

    auto all_vehicles = coupling_graph.getVehicles();
    if(stopFlag){
        cpm::Logging::Instance().write(1, "Received stop before waiting");
    }

    while (!stop_waiting) {
        auto samples = reader_laneGraphTrajectory->take();
        for(auto sample : samples) {
            uint64_t t_message = sample.data().header().create_stamp().nanoseconds();
            if (sample.info().valid() && t_message == t_real_time) {
                messages_received.insert(sample.data().vehicle_id());
            }
        }

        // Successfully waited, if we have received a message from every vehicle
        success_status = (messages_received == all_vehicles);
        // Stop waiting if either we are successful, or we get a stopFlag
        stop_waiting = success_status || stopFlag;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

#if TIMED
    auto end_time = std::chrono::steady_clock::now();
    auto diff = end_time - start_time;
    std::cout << "TIMING: wait_for_other_vehicles took " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
#endif

    return success_status;
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
void VehicleTrajectoryPlanner::write_trajectory( LaneGraphTrajectory trajectory ) {

    // Add TimeStamp to each point
    for( unsigned int i=0; i<trajectory.lane_graph_positions().size(); i++ ) {
        // Calculate, which point in time this index corresponds to
        uint64_t eta = t_real_time + (trajectoryPlan->get_dt_speed_profile_nanos())*i;
        trajectory.lane_graph_positions()[i].estimated_arrival_time().nanoseconds(eta);
    }

    LaneGraphTrajectory msg;
    std::vector<LaneGraphPosition> shortened_trajectory;
    
    int current_edge_index = -1;
    LaneGraphPosition current_position;

    for( unsigned int i=0; i<trajectory.lane_graph_positions().size(); i++ ) {
        current_position = trajectory.lane_graph_positions()[i];

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
    
    msg.vehicle_id(trajectoryPlan->get_vehicle_id());
    msg.header().create_stamp().nanoseconds(t_real_time);
    msg.header().valid_after_stamp().nanoseconds(t_real_time + 5000000000ull);
    this->writer_laneGraphTrajectory->write(msg);
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
    for (std::vector<TrajectoryPoint>::iterator tp_it = trajectory_point_buffer.begin() + 1; tp_it != trajectory_point_buffer.end(); ++tp_it)
    {
        if (tp_it->t().nanoseconds() > t_real_time && it_t_now == trajectory_point_buffer.end())
        {
            it_t_now = tp_it-1;
        }
    }

    trajectory_point_buffer.erase(it_t_now, trajectory_point_buffer.end());
}

void VehicleTrajectoryPlanner::set_writer(
        std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectory> > writer){
    writer_laneGraphTrajectory = writer;
}
void VehicleTrajectoryPlanner::set_reader(
        std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectory> > reader){
    reader_laneGraphTrajectory = reader;
}

/*
 * Prints contents of other_vehicles_buffer to stdout.
 */
void VehicleTrajectoryPlanner::debug_writeOutReceivedTrajectories() {
    for(auto vehicle : other_vehicles_buffer) {
        std::cout << "Vehicle " << static_cast<uint32_t>(vehicle.first) << std::endl;
        for(auto point : vehicle.second) {
            std::cout << point.first << ", "; 
            std::cout << point.second.first << ", "; 
            std::cout << point.second.second << std::endl; 
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
    std::cout <<trajectory_point_buffer[0].t().nanoseconds() << ":\t" <<  trajectory_point_buffer[0].px() << ",\t" << trajectory_point_buffer[0].py() << std::endl;
    for( unsigned int i=1; i<trajectory_point_buffer.size(); i++ ) {
        // This int is currently signed, because it could be negative, but it shouldn't be
	    //std::cout << std::endl;
        double distance = sqrt(pow(((double)trajectory_point_buffer[i].px() - (double)prev_point.px()),2) + pow(((double)trajectory_point_buffer[i].py() - (double)prev_point.py()),2));
	    int64_t time_diff = (int64_t) trajectory_point_buffer[i].t().nanoseconds() - (int64_t) prev_point.t().nanoseconds();
        std::cout <<trajectory_point_buffer[i].t().nanoseconds() << ":\t" <<  trajectory_point_buffer[i].px() << ",\t" << trajectory_point_buffer[i].py();
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
