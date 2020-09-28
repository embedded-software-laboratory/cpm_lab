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
        uint64_t t_planning = 0;

        {
            std::lock_guard<std::mutex> lock(mutex); 
            t_planning = t_real_time;
        }

        while(1)
        {
            // Priority based collision avoidance: Every vehicle avoids 
            // the 'previous' vehicles, in this example those with a smaller ID.
            bool is_collision_avoidable = false;

            this->read_previous_vehicles(t_planning);

            std::cout << "Timestep " << t_planning << std::endl;
            is_collision_avoidable = trajectoryPlan->avoid_collisions(previous_vehicles_buffer);

            if (!is_collision_avoidable){
                cpm::Logging::Instance().write(1,
                        "Found unavoidable collision");
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

                if(t_start == 0)
                {
                    t_start = t_real_time + 2000000000ull;
                    auto trajectory_point = trajectoryPlan->get_trajectory_point();
                    trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_real_time);
                    trajectory_point_buffer.push_back(trajectory_point);

                    //TODO: Change this, so the initial trajectory gets send as well
                    old_lane_graph_trajectory = lane_graph_trajectory;
                }

                vector<LaneGraphTrajectoryChanges> lane_graph_trajectory_changes = get_changes(
                        old_lane_graph_trajectory,
                        lane_graph_trajectory
                        );

                write_changes( lane_graph_trajectory_changes, t_planning );

                // We got our changes to the trajectory; so this is now the old trajectory of our next step
                old_lane_graph_trajectory = lane_graph_trajectory;

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

            shift_previous_vehicles_buffer();

            while(t_real_time + 6000000000ull < t_planning) usleep(110000);
        }
    });

}

void VehicleTrajectoryPlanner::read_previous_vehicles(uint64_t t_planning)
{
    assert(started);

    dds::sub::LoanedSamples<LaneGraphTrajectoryChanges> samples = reader_laneGraphTrajectoryChanges->take();
    for(auto sample : samples) {
        if (sample.info().valid()) {
            // We ignore everything with lower priorities
            if (sample.data().vehicle_id() >= trajectoryPlan->get_vehicle_id()){ continue; }

            // Calculate, which index the timestamp of the received message corresponds to
            uint64_t time_diff = t_planning - sample.data().header().create_stamp().nanoseconds();
            size_t index_offset = time_diff / dt_nanos;

            for ( auto change : sample.data().lane_graph_position_changes() ) {
                if( change.index() - 25*index_offset > 0 ) {
                    previous_vehicles_buffer[sample.data().vehicle_id()][change.index() - 25*index_offset] =
                        std::make_pair(
                            change.lane_graph_position().edge_index(),
                            change.lane_graph_position().edge_path_index()
                            );
                }
            }
        }
    }
}

/*
 * Calculate the LaneGraphTrajectoryChanges between two LaneGraphTrajectories
 * and divide it into messages of suitable length.
 * Currently, RTI DDS messages can only contain 100 elements.
 *
 * This can be changed to not use LaneGraphTrajectory, and then
 * we could delete the LaneGraphTrajectory IDL object type
 */
vector<LaneGraphTrajectoryChanges> VehicleTrajectoryPlanner::get_changes(
        LaneGraphTrajectory trajectory_old, LaneGraphTrajectory trajectory_new) {
    
    vector<LaneGraphTrajectoryChanges> result;
    uint64_t msg_max_length = 100; // Maximum length of a DDS message

    // 16000000ull is dt_speed_profile_nanos from PlanningState
    int steps = dt_nanos / 16000000ull;

    std::vector<LaneGraphPositionChange> temp;
    
    for(int i=0; i<N_STEPS_SPEED_PROFILE; i++) {

        if ( i%msg_max_length == 0 && i>0 ) {
            // We split the changes into messages of length < 100 each
            LaneGraphTrajectoryChanges msg;
            msg.lane_graph_position_changes(temp);
            result.push_back(msg); 

            temp.clear();
        }

        if ( i < N_STEPS_SPEED_PROFILE - steps && trajectory_old.lane_graph_positions().size() > 0 ) {
            LaneGraphPosition pos_old = trajectory_old.lane_graph_positions()[i+steps];
            LaneGraphPosition pos_new = trajectory_new.lane_graph_positions()[i];

            if(pos_old != pos_new) {
                // This is a change to an existing trajectory point
                LaneGraphPositionChange change;
                change.index(i);
                change.lane_graph_position().edge_index(pos_new.edge_index());
                change.lane_graph_position().edge_path_index(pos_new.edge_path_index());
                temp.push_back(change); 
           } 
        } else {
            // This is a newly added trajectory point or trajectory_old is not initialized yet
            LaneGraphPosition pos_new = trajectory_new.lane_graph_positions()[i];

            LaneGraphPositionChange change;
            change.index(i);
            change.lane_graph_position().edge_index(pos_new.edge_index());
            change.lane_graph_position().edge_path_index(pos_new.edge_path_index());
            temp.push_back(change); 
        }
    }

    // Push the last remaining <100 changes to the result
    LaneGraphTrajectoryChanges msg;
    msg.lane_graph_position_changes(temp);
    result.push_back(msg); 

    return result;
}

/*
 * Publish changes to the planned trajectory to DDS
 * 
 * We are publishing only changes instead of the whole trajectory
 * for data size reasons
 */
void VehicleTrajectoryPlanner::write_changes( vector<LaneGraphTrajectoryChanges> vector_changes, uint64_t t_planning ) {
    for ( uint64_t i = 0; i < vector_changes.size(); i++ ) {
        LaneGraphTrajectoryChanges msg = vector_changes[i];
        msg.vehicle_id(trajectoryPlan->get_vehicle_id());
        msg.header().create_stamp().nanoseconds(t_planning);
        msg.header().valid_after_stamp().nanoseconds(t_planning + 1000000000ull);
        this->writer_laneGraphTrajectoryChanges->write(msg);
    }
}

/*
 * Shift our buffer of vehicles forward, because after each timestep
 * index 0 is supposed to be the next planned point
 */
void VehicleTrajectoryPlanner::shift_previous_vehicles_buffer() {
    //TODO: 25 is dt_nanos divided by dt_speed_steps_nanos (magic number, needs changing)
    uint64_t offset_per_timestep = 25;

    for(auto iter_vehicle = previous_vehicles_buffer.begin(); iter_vehicle != previous_vehicles_buffer.end(); ++iter_vehicle){
        std::map<size_t, std::pair<size_t, size_t>> trajectory = iter_vehicle->second;

        for( auto iter_trajectory = trajectory.begin(); iter_trajectory != trajectory.end(); ++iter_trajectory ) {
            if( iter_trajectory->first < offset_per_timestep ) { continue; }  
            previous_vehicles_buffer[iter_vehicle->first][iter_trajectory->first - offset_per_timestep] =
                iter_trajectory->second;
            previous_vehicles_buffer[iter_vehicle->first].erase( iter_trajectory->first );
        }
    }
}

void VehicleTrajectoryPlanner::set_writer(
        std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectoryChanges> > writer){
    writer_laneGraphTrajectoryChanges = writer;
}
void VehicleTrajectoryPlanner::set_reader(
        std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectoryChanges> > reader){
    reader_laneGraphTrajectoryChanges = reader;
}
