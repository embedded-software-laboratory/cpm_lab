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

void VehicleTrajectoryPlanner::start()
{
    assert(!started);

    started = true;

    cpm::Logging::Instance().write(
            3,
            "Starting VehicleTrajectoryPlanner");

    planning_thread = std::thread([this](){
        t_planning = 0;

        {
            std::lock_guard<std::mutex> lock(mutex); 
            t_planning = t_real_time;
        }

        while(1)
        {
            // Priority based collision avoidance: Every vehicle avoids 
            // the 'previous' vehicles, in this example those with a smaller ID.
            bool is_collision_avoidable = false;

            this->read_other_vehicles();

            is_collision_avoidable = trajectoryPlan->avoid_collisions(other_vehicles_buffer);

            std::cout << "TimeStep:" << t_planning << std::endl;
            for( auto element : other_vehicles_buffer ) {
                //std::cout << element.first << std::endl;
                for( auto point : element.second ) {
                    //std::cout << point.first << ":" << point.second.first << "," << point.second.second << std::endl; 
                } 
            }

            if (!is_collision_avoidable){
                cpm::Logging::Instance().write(1,
                        "Found unavoidable collision");
                crashed = true;
                started = false; // end planning
                break;
            } 

            {
                std::lock_guard<std::mutex> lock(mutex); 

                // Get our current trajectory
                LaneGraphTrajectory lane_graph_trajectory;
                trajectoryPlan->get_lane_graph_positions(
                        &lane_graph_trajectory
                        );

                write_trajectory(lane_graph_trajectory);

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
            }

            trajectoryPlan->apply_timestep(dt_nanos);

            t_planning += dt_nanos;

            // Sleep until we need to continue planning
            while(t_real_time + 5*dt_nanos < t_planning) usleep(110000);
        }
    });

}

void VehicleTrajectoryPlanner::read_other_vehicles()
{
    assert(started);

    dds::sub::LoanedSamples<LaneGraphTrajectory> samples = reader_laneGraphTrajectory->take();
    for(auto sample : samples) {
        if (sample.info().valid()) {
            // We ignore everything with lower priorities
            if (sample.data().vehicle_id() >= trajectoryPlan->get_vehicle_id()){ continue; }

            // Reset buffer for this vehicle
            other_vehicles_buffer[sample.data().vehicle_id()].clear();

            //FIXME: After a restart, there temporarily is an unrealistically large offset
            //if(index_offset > 100) {
            //    std::cout << "Received data with unfeasible timestamp";
            //}

            // Save all received positions that are not in the past already
            for ( auto position : sample.data().lane_graph_positions() ) {

                /* Check TimeStamp of each Position to see where it fits into our buffer
                   The "... - dt_nanos" follows from practical observations, where
                   indices were shifted by 1 dt_nanos
                */
                int index_to_use = 
                    (position.estimated_arrival_time().nanoseconds() - t_planning - dt_nanos)
                    / (dt_nanos/timesteps_per_planningstep);

                // If index_to_use is negative, estimated_arrival_time is in the past
                if( index_to_use >= 0 ) {
                    other_vehicles_buffer[sample.data().vehicle_id()][index_to_use] =
                        std::make_pair(
                            position.edge_index(),
                            position.edge_path_index()
                            );
                }
            }
        }
    }
}

/*
 * Publish changes to the planned trajectory to DDS
 * 
 * We are publishing only changes instead of the whole trajectory
 * for data size reasons
 */
void VehicleTrajectoryPlanner::write_trajectory( LaneGraphTrajectory trajectory ) {
    LaneGraphTrajectory msg;

    if(trajectory.lane_graph_positions().size() > msg_max_length) {

        // Write first 100 points to separate vector, to create a shorter msg from it
        std::vector<LaneGraphPosition> shortened_trajectory;
        for( int i=0; i<msg_max_length; i++ ) {
            shortened_trajectory.push_back(trajectory.lane_graph_positions()[i]); 
        }
        msg.lane_graph_positions(shortened_trajectory);

    } else {
        msg = trajectory;
    }

    // Add TimeStamp to each point
    for( int i=0; i<msg.lane_graph_positions().size(); i++ ) {
        // Calculate, which point in time this index corresponds to
        uint64_t eta = t_planning + (dt_nanos/timesteps_per_planningstep)*i;
        msg.lane_graph_positions()[i].estimated_arrival_time().nanoseconds(eta);
    }

    
    msg.vehicle_id(trajectoryPlan->get_vehicle_id());
    msg.header().create_stamp().nanoseconds(t_planning);
    msg.header().valid_after_stamp().nanoseconds(t_planning + 1000000000ull);
    this->writer_laneGraphTrajectory->write(msg);
}

void VehicleTrajectoryPlanner::set_writer(
        std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectory> > writer){
    writer_laneGraphTrajectory = writer;
}
void VehicleTrajectoryPlanner::set_reader(
        std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectory> > reader){
    reader_laneGraphTrajectory = reader;
}
