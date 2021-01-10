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

#include "VehicleTrajectoryPlanner.hpp"



VehicleTrajectoryPlanner::VehicleTrajectoryPlanner(uint64_t dt_nanos):dt_nanos(dt_nanos){}

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

std::unique_ptr<VehicleCommandTrajectory> VehicleTrajectoryPlanner::plan(uint64_t t)
{
    isStopped = false;
    t_real_time = t;

    started = true;
    // Read LaneGraphTrajectory messages and write them into our buffer
    this->read_other_vehicles(); // Waits until we received the correct messages

    // Check if we should stop and return early if we do
    if( stopFlag ) {
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }

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
        t_start = t_real_time + 2000000000ull;
        //auto trajectory_point = trajectoryPlan->get_trajectory_point();
        //trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_start);
        //trajectory_point_buffer.push_back(trajectory_point);
    }
    auto trajectory_point = trajectoryPlan->get_trajectory_point();
    trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_start);
    trajectory_point_buffer.push_back(trajectory_point);

    // Limits size of our trajectory_point_buffer
    while(trajectory_point_buffer.size() > 50)
    {
        trajectory_point_buffer.erase(trajectory_point_buffer.begin());
    }

    if(stopFlag) {
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }
    cpm::Logging::Instance().write(1,
            "Finished planning part"
            );

    // Get our current trajectory
    LaneGraphTrajectory lane_graph_trajectory;
    trajectoryPlan->get_lane_graph_positions(
            &lane_graph_trajectory
    );

    if(stopFlag) {
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }

    // Advance trajectoryPlanningState by 1 timestep
    trajectoryPlan->apply_timestep(dt_nanos);

    // Publish our planned trajectory with other vehicles
    write_trajectory(lane_graph_trajectory);

    /*
    // Useful debugging tool if you suspect that trajectories aren't in sync between vehicles
    std::cout << "Time " << t_real_time << std::endl;
    debug_writeOutReceivedTrajectories();
    trajectoryPlan->debug_writeOutOwnTrajectory();
    */

    isStopped = true;
    return std::unique_ptr<VehicleCommandTrajectory>(new VehicleCommandTrajectory(get_trajectory_command(t_real_time)));
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

    // Clear buffer from previous timestep
    other_vehicles_buffer.clear();

    // Checklist to check, which messages we need to wait for
    vector<bool> checklist = comm_graph[trajectoryPlan->get_vehicle_id()-1];

    bool still_waiting = true;
    while( still_waiting && !stopFlag) {
        auto samples = reader_laneGraphTrajectory->take();
        for(auto sample : samples) {
            LaneGraphTrajectory data = sample.data();
            uint64_t t_message = data.header().create_stamp().nanoseconds();

            if (sample.info().valid() && t_message == t_real_time) {
                if ( !checklist.at(data.vehicle_id()-1) ){
                    // Ignore message if the vehicle is not in our checklist
                    continue;
                } else {
                    // If vehicle is in our checklist, remove/"check" it after receiving a message
                    checklist[data.vehicle_id()-1] = false;
                }
                
                int prev_buffer_index = -1;
                int prev_edge_index = -1;
                int prev_edge_path_index = -1;

                // Save all received positions that are not in the past already
                for ( LaneGraphPosition position : data.lane_graph_positions() ) {


                    // Check TimeStamp of each Position to see where it fits into our buffer
                    // Casting to signed number because we might get negative values
                    long long t_eta = position.estimated_arrival_time().nanoseconds();
                    long long dt_minor_timestep = dt_nanos/timesteps_per_planningstep;
                    int index = 
                        ( t_eta - (long long) t_real_time)
                        / dt_minor_timestep;

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
        still_waiting = false;
        for ( bool entry : checklist ) {
            if( entry ) {
                still_waiting = true;
            }
        }

    }
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
    std::cout << "Stopped planning early at time " << t_real_time << std::endl;

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
        uint64_t eta = t_real_time + (dt_nanos/timesteps_per_planningstep)*i;
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
 * comm_graph is for determining the order of planning
 */
void VehicleTrajectoryPlanner::set_comm_graph(
        vector<vector<bool>> matrix) {
    comm_graph = matrix;
}

void VehicleTrajectoryPlanner::set_writer(
        std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectory> > writer){
    writer_laneGraphTrajectory = writer;
}
void VehicleTrajectoryPlanner::set_reader(
        std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectory> > reader){
    reader_laneGraphTrajectory = reader;
}

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
