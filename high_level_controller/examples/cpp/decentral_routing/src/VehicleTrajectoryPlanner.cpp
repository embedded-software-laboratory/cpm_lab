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

            this->read_previous_vehicles();

            is_collision_avoidable = trajectoryPlan->avoid_collisions(previous_vehicles_buffer);

            if (!is_collision_avoidable){
                cpm::Logging::Instance().write(1,
                        "Found unavoidable collision");
                started = false; // end planning
                break;
            } 

            {
                std::lock_guard<std::mutex> lock(mutex); 

                //cpm::Logging::Instance().write(3,
                //        "About to publish LaneGraphTrajectory");
                LaneGraphTrajectory lane_graph_trajectory;
                trajectoryPlan->get_lane_graph_positions(
                        &lane_graph_trajectory
                        );

                // Is t_planning the correct time to use for this? I do not know
                lane_graph_trajectory.header().create_stamp().nanoseconds(t_planning);
                lane_graph_trajectory.header().valid_after_stamp().nanoseconds(t_planning + 1000000000ull);
                this->writer_laneGraphTrajectory->write(lane_graph_trajectory);

                if(t_start == 0)
                {
                    t_start = t_real_time + 2000000000ull;
                    auto trajectory_point = trajectoryPlan->get_trajectory_point();
                    trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_real_time);
                    trajectory_point_buffer.push_back(trajectory_point);

                    //TODO: Change this, so the initial trajectory gets send as well
                    prev_lane_graph_trajectory = lane_graph_trajectory;
                }

                vector<LaneGraphTrajectoryChanges> lane_graph_trajectory_changes = get_changes(
                        prev_lane_graph_trajectory,
                        lane_graph_trajectory
                        );

                write_changes( lane_graph_trajectory_changes, t_planning );

                // We got our changes to the trajectory; so this is now the old trajectory of our next step
                prev_lane_graph_trajectory = lane_graph_trajectory;

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

void VehicleTrajectoryPlanner::read_previous_vehicles()
{
    assert(started);
    dds::sub::LoanedSamples<LaneGraphTrajectory> samples = reader_laneGraphTrajectory->take();
    for (auto sample : samples) {
        if (sample.info().valid()) {
            // We ignore everything with lower priorities
            if (sample.data().vehicle_id() >= trajectoryPlan->get_vehicle_id()){ continue; }
            previous_vehicles_buffer[sample.data().vehicle_id()] = sample.data();
        }
    }

    dds::sub::LoanedSamples<LaneGraphTrajectoryChanges> samples2 = reader_laneGraphTrajectoryChanges->take();
    for(auto sample : samples2) {
        if (sample.info().valid()) {
            // We ignore everything with lower priorities
            if (sample.data().vehicle_id() >= trajectoryPlan->get_vehicle_id()){ continue; }

            if( sample.data().lane_graph_position_changes().size() > 0 ) { 
                std::cout << "Inputting " << sample.data().lane_graph_position_changes().size() << " changes" << std::endl;
            }
            for ( auto change : sample.data().lane_graph_position_changes() ) {
                previous_vehicles_buffer2[sample.data().vehicle_id()][change.index()] =
                    std::make_pair(
                        change.lane_graph_position().edge_index(),
                        change.lane_graph_position().edge_path_index()
                        );
            }
        }
    }
    for(auto it = previous_vehicles_buffer2.begin(); it != previous_vehicles_buffer2.end(); ++it){
        //std::cout << "Key: " << (int) it->first << std::endl;
    }
}

vector<LaneGraphTrajectoryChanges> VehicleTrajectoryPlanner::get_changes(
        LaneGraphTrajectory trajectory_old, LaneGraphTrajectory trajectory_new) {
    
    vector<LaneGraphTrajectoryChanges> result;
    uint64_t msg_max_length = 100; // Maximum length of a DDS message

    // 16000000ull is dt_speed_profile_nanos from PlanningState
    int steps = dt_nanos / 16000000ull;
    //TODO: this hardcoded 100 is N_STEPS_SPEED_PROFILE and should NOT be hardcoded
    int n_steps_speed_profile = 100;

    std::vector<LaneGraphPositionChange> temp;
    
    for(int i=0; i<n_steps_speed_profile - steps; i++) {

        if ( i%msg_max_length == 0 && i>0 ) {
            // We split the changes into messages of length < 100 each
            LaneGraphTrajectoryChanges msg;
            msg.lane_graph_position_changes(temp);
            result.push_back(msg); 

            std::vector<LaneGraphPositionChange> temp;
        }

        LaneGraphPosition pos_old = trajectory_old.lane_graph_positions()[i+steps];
        LaneGraphPosition pos_new = trajectory_new.lane_graph_positions()[i];

        if(pos_old != pos_new) {
            //std::cout << "Change at Index: " << i << "; Old: " << pos_old << "; New: " << pos_new << std::endl;

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

void VehicleTrajectoryPlanner::write_changes( vector<LaneGraphTrajectoryChanges> vector_changes, uint64_t t_planning ) {
    for ( uint64_t i = 0; i < vector_changes.size(); i++ ) {
        LaneGraphTrajectoryChanges msg = vector_changes[i];
        msg.vehicle_id(trajectoryPlan->get_vehicle_id());
        msg.header().create_stamp().nanoseconds(t_planning);
        msg.header().valid_after_stamp().nanoseconds(t_planning + 1000000000ull);
        this->writer_laneGraphTrajectoryChanges->write(msg);
    }
}

void VehicleTrajectoryPlanner::shift_previous_vehicles_buffer() {

    uint64_t offset_per_timestep = 25;
    for(auto iter_vehicle = previous_vehicles_buffer2.begin(); iter_vehicle != previous_vehicles_buffer2.end(); ++iter_vehicle){
        std::map<size_t, std::pair<size_t, size_t>> trajectory = iter_vehicle->second;

        for( auto iter_trajectory = trajectory.begin(); iter_trajectory != trajectory.end(); ++iter_trajectory ) {
            //TODO: 25 is dt_nanos divided by dt_speed_steps_nanos
            if( iter_trajectory->first < offset_per_timestep ) { continue; }  
            previous_vehicles_buffer2[ iter_trajectory->first - offset_per_timestep ] =
                previous_vehicles_buffer2[ iter_trajectory->first ];
        }
    }
}

void VehicleTrajectoryPlanner::set_writer(
        std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectory> > writer){
    writer_laneGraphTrajectory = writer;
}
void VehicleTrajectoryPlanner::set_reader(
        std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectory> > reader){
    reader_laneGraphTrajectory = reader;
}

//TODO: Switch everything from LaneGraphTrajectory to LaneGraphTrajectoryChanges
void VehicleTrajectoryPlanner::set_writer2(
        std::shared_ptr< dds::pub::DataWriter<LaneGraphTrajectoryChanges> > writer){
    writer_laneGraphTrajectoryChanges = writer;
}
void VehicleTrajectoryPlanner::set_reader2(
        std::shared_ptr< dds::sub::DataReader<LaneGraphTrajectoryChanges> > reader){
    reader_laneGraphTrajectoryChanges = reader;
}
