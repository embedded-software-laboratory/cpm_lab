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

void VehicleTrajectoryPlanner::plan(uint64_t t)
{
    t_real_time = t;
    if(!started) {
        t_planning = t_real_time;
    }

    started = true;
    // Read LaneGraphTrajectory messages and write them into our buffer
    // TODO: This method could block until we have the right responses
    cpm::Logging::Instance().write(1,
            "Planner %d waiting to start planning %lu",
            trajectoryPlan->get_vehicle_id(),
            t_real_time
            );
    this->read_other_vehicles();
    cpm::Logging::Instance().write(1,
            "Planner %d starting planning %lu",
            trajectoryPlan->get_vehicle_id(),
            t_real_time
            );

    while(t_planning < t_real_time + planning_horizont*dt_nanos)
    {
        // Priority based collision avoidance: Every vehicle avoids 
        // the 'previous' vehicles, in this example those with a smaller ID.
        bool is_collision_avoidable = false;

        is_collision_avoidable = trajectoryPlan->avoid_collisions(other_vehicles_buffer);

        if (!is_collision_avoidable){
            cpm::Logging::Instance().write(1,
                    "Found unavoidable collision");
            crashed = true;
            started = false; // end planning
            break;
        } 


        if(t_start == 0)
        {
            t_start = t_real_time + 2000000000ull;
            auto trajectory_point = trajectoryPlan->get_trajectory_point();
            trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_real_time);
            trajectory_point_buffer.push_back(trajectory_point);
        }

        while(trajectory_point_buffer.size() > 50)
        {
            trajectory_point_buffer.erase(trajectory_point_buffer.begin());
        }
        auto trajectory_point = trajectoryPlan->get_trajectory_point();
        trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_start);
        trajectory_point_buffer.push_back(trajectory_point);

        trajectoryPlan->apply_timestep(dt_nanos);

        t_planning += dt_nanos;
    }

    // Get our current trajectory
    LaneGraphTrajectory lane_graph_trajectory;
    trajectoryPlan->get_lane_graph_positions(
            &lane_graph_trajectory
    );

    cpm::Logging::Instance().write(1,
            "Planner %d finished planning",
            trajectoryPlan->get_vehicle_id()
            );

    write_trajectory(lane_graph_trajectory);
}

void VehicleTrajectoryPlanner::read_other_vehicles()
{
    assert(started);

    // Clear buffer from previous timestep
    // FIXME: read_other_vehicles is executed more than once per timestep.
    // We probably should clear the buffer each time
    other_vehicles_buffer.clear();

    vector<bool> checklist = comm_graph[trajectoryPlan->get_vehicle_id()-1];
    bool still_waiting = true;

    while( still_waiting ) {
        dds::sub::LoanedSamples<LaneGraphTrajectory> samples = reader_laneGraphTrajectory->take();
        for(auto sample : samples) {
            std::cout << "Header:" << sample.data().header().create_stamp().nanoseconds() << std::endl;
            std::cout << "Expect:" <<t_real_time << std::endl;
            if (sample.info().valid() &&
                    // Check if this is actually a new message
                    // Diese if-Bedingung verhindert, dass Fahrzeug 2 & 3 starten
                    // Grund: Wir verpassen anscheinend die erste Nachricht von Fahrzeug 1
                    sample.data().header().create_stamp().nanoseconds() == t_real_time) {
                // We ignore everything not in our comm_graph
                if ( !checklist.at(sample.data().vehicle_id()-1) ){
                    continue;
                } else {
                    checklist[sample.data().vehicle_id()-1] = false;
                }
                
                int prev_buffer_index = -1;
                int prev_edge_index = -1;
                int prev_edge_path_index = -1;

                // Save all received positions that are not in the past already
                for ( auto position : sample.data().lane_graph_positions() ) {


                    // Check TimeStamp of each Position to see where it fits into our buffer
                    int index = 
                        ((long long) position.estimated_arrival_time().nanoseconds() - (long long) t_planning)
                        / ((long long) dt_nanos/timesteps_per_planningstep);

                    // We sometimes get unrealistic indices, which we don't want to use
                    if( index < -100 ) {
                        continue;
                    }

                    // If index_to_use is negative, estimated_arrival_time is in the past
                    if( index >= 0 ) {
                        other_vehicles_buffer[sample.data().vehicle_id()][index] =
                            std::make_pair(
                                position.edge_index(),
                                position.edge_path_index()
                            );
                    }

                    // If the prev_... vars are set, interpolate using them
                    // We cannot interpolate if there is just one point in the buffer
                    if( prev_edge_index >= 0 ) {
                        interpolate_other_vehicles_buffer(
                                sample.data().vehicle_id(),
                                prev_buffer_index, prev_edge_index, prev_edge_path_index,
                                index, position.edge_index(), position.edge_path_index()
                        );
                    }

                    // Remember these for interpolation
                    prev_buffer_index = index;
                    prev_edge_index = position.edge_index();
                    prev_edge_path_index = position.edge_path_index();
                    
                }
            }
        }

        still_waiting = false;
        for ( bool entry : checklist ) {
            if( entry ) {
                still_waiting = true;
            }
        }

    }
}

/*
 * Publish the planned trajectory to DDS
 */
void VehicleTrajectoryPlanner::write_trajectory( LaneGraphTrajectory trajectory ) {

    // Add TimeStamp to each point

    for( unsigned int i=0; i<trajectory.lane_graph_positions().size(); i++ ) {
        // Calculate, which point in time this index corresponds to
        uint64_t eta = t_planning + (dt_nanos/timesteps_per_planningstep)*i;
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
    msg.header().create_stamp().nanoseconds(t_planning);
    msg.header().valid_after_stamp().nanoseconds(t_planning + 1000000000ull);
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
