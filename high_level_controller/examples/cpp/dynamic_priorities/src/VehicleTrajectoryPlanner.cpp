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
#define TIMED true

/**
 * \brief A HLC that runs distributedly and plans for the vehicles using priority based planning. It can update its priorities distributedly to increase feasibility. Feature a Future Collision Based and a random heuristic.
 * It is also able to plan using static priorities. 
 * \ingroup dynamic_priorities
 */
#define T_START_DELAY_NANOS 1000000000ull

#include "VehicleTrajectoryPlanner.hpp"
#include <cstdlib>
#include <string.h>
#include <iostream>
#include <sstream>



/**
 * \file VehicleTrajectoryPlanner.cpp
 * \ingroup distributed_routing
 */

VehicleTrajectoryPlanner::VehicleTrajectoryPlanner(PriorityMode _mode){
    writer_sync = std::unique_ptr<cpm::Writer<FallbackSync>>(
        new cpm::Writer<FallbackSync>("fallbacksync")
        );
    reader_sync =  std::unique_ptr<cpm::ReaderAbstract<FallbackSync>>(
        new cpm::ReaderAbstract<FallbackSync>("fallbacksync")
        );
    mode = _mode;
}

VehicleCommandTrajectory VehicleTrajectoryPlanner::get_trajectory_command(uint64_t t_now)
{
    VehicleCommandTrajectory vehicleCommandTrajectory;
    vehicleCommandTrajectory.vehicle_id(trajectoryPlan->get_vehicle_id());
    vehicleCommandTrajectory.trajectory_points(
            rti::core::vector<TrajectoryPoint>(trajectory_point_buffer)
    );
    vehicleCommandTrajectory.header().create_stamp().nanoseconds(t_now); //You just need to set t_now here, as it was created at t_now
    vehicleCommandTrajectory.header().valid_after_stamp().nanoseconds(t_now + T_START_DELAY_NANOS); //Hardcoded value from the planner (t_start), should be correct (this value should correlate with the trajectory point that should be valid at t_now)

    return vehicleCommandTrajectory;
}

void VehicleTrajectoryPlanner::set_vehicle(std::unique_ptr<VehicleTrajectoryPlanningState> vehicle)
{
    assert(!started);
    cpm::Logging::Instance().write(1,
            "setting vehicle");
    trajectoryPlan = std::move(vehicle);
    std::cout << "Priority assignment strategy: " << static_cast<int>(mode) << std::endl;
    if (mode == PriorityMode::random)
    {
        std::srand(trajectoryPlan->get_vehicle_id());
    }
    vehicle_id = trajectoryPlan->get_vehicle_id();

    for (auto vehicle_id : coupling_graph.getVehicles())
    {
        prio_vec.push_back(vehicle_id);
    }

    // Open file to write evaluation data
    evaluation_stream.open(("evaluation_" +  std::to_string((int)vehicle_id) + ".csv"));

    if (evaluation_stream.is_open())
    {
        std::cout << "opened eval file" << std::endl;
    }
    evaluation_stream << "id:" << (int)vehicle_id << std::endl;
    evaluation_stream << "time;speed_profile;fca;new_old_fallback_prio;priority_vector;trajectory" << std::endl;
}

std::unique_ptr<VehicleCommandTrajectory> VehicleTrajectoryPlanner::plan(uint64_t t, uint64_t dt)
{

    trajectoryPlan->print_speed_profile();
    auto start_time = std::chrono::steady_clock::now();
    isStopped = false;
    t_real_time = t;
    dt_nanos = dt;

    // Timesteps should come in a logical order
    assert(t_prev < t_real_time);

    // Catch up planningState if we missed a timestep
    while (t_real_time - t_prev > dt && t_prev != 0)
    {
        trajectoryPlan->apply_timestep(dt_nanos);
        t_prev += dt_nanos;
    }
    t_prev = t_real_time;
    started = true;

    std::cout << "STEP: " << t_real_time << "; t:" << t << "; dt:" << dt << std::endl;
    evaluation_stream << t_real_time << "; "; 
    // TODO ps this is "fixed" speed profile? check time
    trajectoryPlan->write_current_speed_profile(evaluation_stream, dt_nanos);


    trajectoryPlan->save_speed_profile();
    trajectoryPlan->reset_speed_profile();
    
    bool new_prios_feasible = false;
    bool old_prios_feasible = false;

    switch(mode)
    {
    case PriorityMode::fca :
    {
        // updates prios and plans based on fca only if its feasible
        new_prios_feasible = plan_fca_priorities();
        bool other_feasible = synchronise(coupling_graph.getVehicles(), new_prios_feasible, 1);
        new_prios_feasible = new_prios_feasible && other_feasible;
        std::cout << "synchronised" << std::endl;
        break;
    }


    case PriorityMode::random :
    {
        new_prios_feasible = plan_random_priorities();
        break;
    }


    case PriorityMode::id :
    {
        // TODO ps reset_speed_profile can be avoided if no collisions in the
        //         final timestep are avoided.
        old_prios_feasible = plan_static_priorities();
        evaluation_stream << 0 << ";"; // fca value
        break;
    }


    case PriorityMode::vertex_ordering :
    {
        plan_vertex_ordering_priorities();
        break;
    }
    }

    // Check result
    if (new_prios_feasible)
    {
        evaluation_stream << 0 << ";";
    } 
    // if dynamic prio, but change not feasible, try if old prios work
    else
    {
        if (mode != PriorityMode::id)
        {
            std::cout << "Using old prios " << std::endl;
            trajectoryPlan->reset_speed_profile();
            // plan with old priorities
            old_prios_feasible = plan_static_priorities();
        }
        
        // if old prios don't work just extend the halt at the end of the horizon
        if (!old_prios_feasible)
        {
            std::cout << "Old prios infeasible: " << std::endl;
            evaluation_stream << 2 << ";";
            trajectoryPlan->revert_speed_profile();
        }
        else
        {
            evaluation_stream << 1 << ";";
        }
    }

    
    int coll_left = trajectoryPlan->potential_collisions(other_vehicles_buffer, false);
    evaluation_stream << "col_left " << coll_left << ";";
    if (coll_left > 0)
    {
        std::cout << "we still have: " << coll_left << " collisions" << std::endl;
    }
    trajectoryPlan->print_speed_profile();
    //trajectoryPlan->debug_writeOutOwnTrajectory();

    for (auto  prio : prio_vec)
    {
        evaluation_stream << (int)prio << ",";
    }
    evaluation_stream << ";";

    // compute the trajectory point vector which will be send to the vehicle
    if (!stopFlag)
    {
        if (t_start == 0)
        {
            t_start = t_real_time;
        }
        else
        {
            clear_past_trajectory_point_buffer();
        }

        // Get new points from PlanningState
        uint n_trajectory_points = 6;
        std::vector<TrajectoryPoint> new_trajectory_points = trajectoryPlan->get_planned_trajectory(n_trajectory_points-1, dt_nanos);
        for (auto &point : new_trajectory_points)
        {
            point.t().nanoseconds(point.t().nanoseconds() + t_start + T_START_DELAY_NANOS);
            evaluation_stream << "x" <<point.px() << ",y" << point.py() << ",vx" << point.vx() << ",vy" << point.vy() <<",";
        }
        //evaluation_stream << ";";

        // Append points to trajectory_point_buffer
        trajectory_point_buffer.insert(
            trajectory_point_buffer.end(),
            new_trajectory_points.begin(),
            new_trajectory_points.end());

        // Limit size of trajectory buffer; delete oldest points first;
        while (trajectory_point_buffer.size() > n_trajectory_points)
        {
            trajectory_point_buffer.erase(trajectory_point_buffer.begin());
        }
    }
    //debug_writeOutReceivedTrajectories();
    //debug_analyzeTrajectoryPointBuffer();

    // Advance trajectoryPlanningState by 1 timestep
    trajectoryPlan->apply_timestep(dt_nanos);
    auto end_time = std::chrono::steady_clock::now();
    auto diff = end_time - start_time;
    std::cout << "TIMING: plan step ";
    std::cout
        << "took "
        << std::chrono::duration<double, std::milli>(diff).count()
        << " ms"
        << std::endl;

    // flush evaluation buffer line
    evaluation_stream  << std::endl;
    if (!stopFlag)
    {
        isStopped = true;
        no_trajectory_counter = 0;
        return std::unique_ptr<VehicleCommandTrajectory>(
            new VehicleCommandTrajectory(get_trajectory_command(t_real_time)));
    }
    else
    {
        // middleware set stopFlag
        cpm::Logging::Instance().write(2,
                                       "%lu: Not returning trajectory", t_real_time);
        no_trajectory_counter++;
        // If no trajectory was sent for 500ms, vehicles stop
        // Except for the first , like, 4 seconds, where it's normal
        if (no_trajectory_counter * dt_nanos > 500000000ull && (t_real_time - t_start > 4000000000ull))
        {
            cpm::Logging::Instance().write(1,
                                           "Planner missed too many timesteps");
            crashed = true;
            started = false; // end planning
            throw std::runtime_error("Planner missed too many timesteps");
        }
        isStopped = true;
        return std::unique_ptr<VehicleCommandTrajectory>(nullptr);
    }
}

bool VehicleTrajectoryPlanner::plan_random_priorities(){
    new_prio_vec.clear();          // save new priorities until we know they are feasible and update prio_vec
    other_vehicles_buffer.clear(); // received trajectories are cleared

    uint16_t rand_fca = std::rand() % 500;
    write_fca(vehicle_id, rand_fca);
    evaluation_stream << (int)rand_fca << ";";
    std::cout << "trying new prios" << std::endl;
    new_prio_vec = fca_prio_vec(rand_fca);

    bool feasible = plan_static_priorities();
    std::cout << "new prios fasible: " << feasible << std::endl;
    bool other_feasible = synchronise(coupling_graph.getVehicles(), feasible, 3);
    feasible = feasible && other_feasible;
    std::cout << "other vehicles fasible: " << feasible << std::endl;
    if (feasible)
    {
        prio_vec = new_prio_vec;
    }

    return feasible;
}

bool VehicleTrajectoryPlanner::plan_static_priorities(){
    bool prios_feasible = true;
    // TODO ps all_received_messages not used? kann weg
    all_received_messages.clear(); // Messages_received gets reset
    other_vehicles_buffer.clear();

    bool other_feasible = !read_vehicles(prev_vehicles(), other_vehicles_buffer, true);
    prios_feasible = prios_feasible && other_feasible; // Waits until we received the trajs needed for planning

    if (prios_feasible)
    {
        bool avoid_successful = trajectoryPlan->avoid_collisions(other_vehicles_buffer);
        bool has_collisions = !avoid_successful;
        prios_feasible = prios_feasible && avoid_successful;
        send_plan_to_hlcs(false, true, has_collisions);

        if (prios_feasible)
        {
            other_feasible = !read_vehicles(ignored_vehicles(), ignored_vehicles_buffer);
            prios_feasible = prios_feasible && other_feasible;
            std::cout << "current prios worked: " << prios_feasible << std::endl;
        }
    }
    return prios_feasible;
}

bool VehicleTrajectoryPlanner::plan_fca_priorities()
{
    bool new_prios_feasible = true; // we initially expect that we can change the prios
    bool avoid_successful = false;
    bool has_collisions = false;


    new_prio_vec.clear();          // save new priorities until we know they are feasible and update prio_vec
    other_vehicles_buffer.clear(); // received trajectories are cleared

    

    send_plan_to_hlcs(true, false);                                                 // send own optimal trajectory
    read_optimal_trajectories();                                                    // read all optimal trajectories
    // TODO ps why graph_based_priorities here? kann weg
    trajectoryPlan->compute_graph_based_priorities(vehicles_buffer);
    uint16_t own_fca = trajectoryPlan->potential_collisions(vehicles_buffer, true); // compute fca based on optimal trajectories
    evaluation_stream << (int)own_fca << ";";
    // all vehicles become active
    active_vehicles = coupling_graph.getVehicles();
    is_active = true;

    while (is_active && new_prios_feasible)
    {
        synchronise(active_vehicles, true, 5);
        write_fca(vehicle_id, own_fca);
        uint8_t winner_id = get_largest_fca(own_fca); // read fca -> winner plans
        if (winner_id == vehicle_id) // if winner plan; send trajectory;
        {
            avoid_successful = trajectoryPlan->avoid_collisions(other_vehicles_buffer); // plan own trajectory
            new_prios_feasible = new_prios_feasible && avoid_successful;
            has_collisions = !avoid_successful;
            send_plan_to_hlcs(false, true, has_collisions); // send out plan
            is_active = false;
        }
        else
        {
            bool winner_feasible = !read_vehicles({winner_id}, other_vehicles_buffer, true); // else read winner and repeat
            new_prios_feasible = new_prios_feasible && winner_feasible;
            if (winner_feasible)
            {
                vehicles_buffer[winner_id] = other_vehicles_buffer[winner_id];
                own_fca = trajectoryPlan->update_potential_collisions(vehicles_buffer, winner_id, own_fca);
            }
        }
        active_vehicles.erase(winner_id);  // winner is not active anymore -> remove from set
        new_prio_vec.push_back(winner_id); // the position in the vector corresponds to the prio
    }

    if (new_prios_feasible)
    {
        bool other_feasible = !read_vehicles(active_vehicles, ignored_vehicles_buffer); // read remaining and check if still feasible
        new_prios_feasible = new_prios_feasible && other_feasible;
    }
    if (new_prios_feasible)
    {
        // inserts the remaining vehicles with lower priority than self. the order doesn't matter.
        new_prio_vec.insert(new_prio_vec.end(), active_vehicles.begin(), active_vehicles.end());
        prio_vec = new_prio_vec; // update prios
        debug_write_prio_vec();
    }

    std::cout << "New fca prios feasible: " << new_prios_feasible << std::endl;
    return new_prios_feasible;
}

bool VehicleTrajectoryPlanner::plan_vertex_ordering_priorities()
{
    new_prio_vec.clear();          // save new priorities until we know they are feasible and update prio_vec
    other_vehicles_buffer.clear(); // received trajectories are cleared

    send_plan_to_hlcs(true, false);                                                 // send own optimal trajectory
    read_optimal_trajectories();                                                    // read all optimal trajectories
    
    std::vector<uint8_t> old_prio_vec = prio_vec;
    prio_vec = trajectoryPlan->compute_graph_based_priorities(vehicles_buffer);
    synchronise(coupling_graph.getVehicles(), true, 5);
    evaluation_stream << 0 << ";";

    bool feasible = plan_static_priorities();
    std::cout << "new prios fasible: " << feasible << std::endl;
    bool other_feasible = synchronise(coupling_graph.getVehicles(), feasible, 3);
    feasible = feasible && other_feasible;
    std::cout << "other vehicles fasible: " << feasible << std::endl;
    if (!feasible)
    {
        std::cout << "using old prios" << std::endl;
        prio_vec = old_prio_vec;
        trajectoryPlan->revert_speed_profile();
        bool old_prio_feasible = plan_static_priorities();
        other_feasible = synchronise(coupling_graph.getVehicles(), old_prio_feasible, 4);
        old_prio_feasible = old_prio_feasible && other_feasible;
        if (!old_prio_feasible)
        {
            std::cout << "stopping" << std::endl;
            trajectoryPlan->revert_speed_profile();
            evaluation_stream << 2 << ";";
        }
        else
        {
            evaluation_stream << 1 << ";";
        }
    } else 
    {
        evaluation_stream << 0 << ";";
    }
    return true;
}

void VehicleTrajectoryPlanner::debug_write_prio_vec(){
    std::cout << "prio vec: ";
    for (auto e : prio_vec)
    {
        std::cout << ", " << (int)e;
    }
    std::cout << std::endl;
}

std::set<uint8_t> VehicleTrajectoryPlanner::prev_vehicles()
{
    std::set<uint8_t> prev_veh;
    prev_veh.insert(prio_vec.begin(), std::find(prio_vec.begin(), prio_vec.end(), vehicle_id));
    return prev_veh;
}

std::set<uint8_t> VehicleTrajectoryPlanner::ignored_vehicles()
{
    std::set<uint8_t> ig_veh;
    ig_veh.insert(std::find(prio_vec.begin(), prio_vec.end(), vehicle_id)+1, prio_vec.end());
    return ig_veh;
}

bool VehicleTrajectoryPlanner::synchronise(std::set<uint8_t> vehicle_ids, bool feasible, uint8_t sync_id){
    FallbackSync sync_message;
    sync_message.vehicle_id(trajectoryPlan->get_vehicle_id());
    sync_message.feasible(feasible);
    sync_message.syncid(sync_id);
    sync_message.header().create_stamp().nanoseconds(t_real_time);
    sync_message.header().valid_after_stamp().nanoseconds(t_real_time + 15000000000ull);
    writer_sync->write(sync_message);

    std::cout << "synced with: ";
    bool other_feasible = true;
    std::set<uint8_t> received_ids;
    while (     !std::includes(received_ids.begin(), received_ids.end(), vehicle_ids.begin(), vehicle_ids.end()) 
            &&  !stopFlag)
    {
        auto messages = reader_sync->take();
        for (auto sync_message : messages)
        {
            if (t_real_time == sync_message.header().create_stamp().nanoseconds() && sync_message.syncid() == sync_id) // if there is a message from the current time the other veh is ready
            {
                received_ids.insert(sync_message.vehicle_id());
                other_feasible = other_feasible && sync_message.feasible();
                std::cout << (int)sync_message.vehicle_id() << " ,";
            }
        }
    }
    std::cout << std::endl;
    return other_feasible;
}

// Read fcas, returns if this vehicle is the winner
uint8_t VehicleTrajectoryPlanner::get_largest_fca(uint16_t own_fca){
    std::set<uint8_t> received_fcas;
    uint8_t winner = trajectoryPlan->get_vehicle_id();
    uint16_t largest_fca = own_fca;

    while( !std::includes(received_fcas.begin(), received_fcas.end(),
                active_vehicles.begin(), active_vehicles.end()) // Becomes true, when we received fca from all active vehicles 
                 && !active_vehicles.empty()
            ) 
    {   
        auto samples = reader_fca->take();
        
        for( auto sample : samples){
            std::cout << "vid: " << (int)vehicle_id << ", fcaid: "<< (int)sample.vehicle_id() << std::endl; 
            if (t_real_time == sample.header().create_stamp().nanoseconds())
            {
                received_fcas.insert(sample.vehicle_id());
                if (sample.fca() > largest_fca || (sample.fca() == largest_fca && sample.vehicle_id() > winner))
                {
                    winner = sample.vehicle_id();
                    largest_fca = sample.fca();
                }
            } 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return winner;
}
/*
 * Reads the trajectories of vehicles with a higher prio
 */
bool VehicleTrajectoryPlanner::read_previous_vehicles() {
    return read_vehicles(coupling_graph.getPreviousVehicles(trajectoryPlan->get_vehicle_id()), other_vehicles_buffer, true, false);
}

/*
 * Reads the planned paths of concurrent planning vehicles
 */
bool VehicleTrajectoryPlanner::read_concurrent_vehicles() {
    return read_vehicles(coupling_graph.getConcurrentVehicles(trajectoryPlan->get_vehicle_id()), other_vehicles_buffer, true, false, false); 
}

/*
 * Waits until all ignored vehicles (with a lower prio) have planned
 */
bool VehicleTrajectoryPlanner::wait_for_ignored_vehicles() {
    return read_vehicles(coupling_graph.getIgnoredVehicles(trajectoryPlan->get_vehicle_id()), ignored_vehicles_buffer, true, true, false);
}

void VehicleTrajectoryPlanner::read_optimal_trajectories(){
    vehicles_buffer.clear();
    set<uint8_t> other_vehicles = coupling_graph.getVehicles();
    //other_vehicles.erase(other_vehicles.find(vehicle_id));
    read_vehicles(other_vehicles, vehicles_buffer, true, false, true);
}


// Reads LaneGraphTrajectories sent by other vehicles in this timestep
// into other_vehicles_buffer.
// Blocks until a message from all vehicles specified in the comm graph
// is received or the stop() method is called.

bool VehicleTrajectoryPlanner::read_vehicles(
    std::set<uint8_t> vehicle_ids, 
    std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer, 
    bool write_to_buffer, 
    bool final_messages_only,
    bool only_optimal)
{
    assert(started);

#if TIMED
    auto start_time = std::chrono::steady_clock::now();
#endif

    // Loop until we receive a stopFlag OR
    // our messages_received contains all vehicles we're waiting for
    std::set<uint8_t> received_messages;
    bool received_collisions = false;
    while( !std::includes(received_messages.begin(), received_messages.end(),
                vehicle_ids.begin(), vehicle_ids.end()) // Becomes true, when we received msg from all vehicle_ids
            && !vehicle_ids.empty() // Stop immediately when vehicle_ids is empty
            && !stopFlag // Stop early, when we receive a stopFlag
            ) {
        auto samples = reader_trajectory->take();
        for(Trajectory sample : samples) {
            uint64_t t_message = sample.header().create_stamp().nanoseconds();

            // Usually we only listen to messages with MessageType Final
            if ( (final_messages_only && sample.type() != MessageType::Final) 
                || (only_optimal && sample.type() != MessageType::Optimal)) {
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

                received_collisions = received_collisions || sample.has_collisions();
                std::cout << "veh " << (int)sample.vehicle_id() << " hascoll: " << sample.has_collisions() << ", ";
                if (sample.has_collisions())
                {
                    return true;
                }
                
                // Only look at the lane graph positions if we need to write them to buffer
                if( !write_to_buffer) {
                    continue;
                }

                // Clear out the buffer for this specific vehicle
                // Especially important for iterative planning vehicles
                vehicles_buffer[sample.vehicle_id()].clear();
                vehicles_buffer[sample.vehicle_id()].reserve(1000);
                std::cout << "receiving: " << (int)sample.vehicle_id();
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

                    // Only write into buffer if index is positive, no interpolation is done since this hlc sends the full speed profiles
                    if( index >= 0 ) {
                        vehicles_buffer[sample.vehicle_id()].push_back(
                            std::make_pair( index,
                                std::make_pair(
                                    position.edge_index(),
                                    position.edge_path_index()
                                )
                            )
                        );
                    }
                }
            }
            std::cout << std::endl;
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
    evaluation_stream.close();
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
void VehicleTrajectoryPlanner::send_plan_to_hlcs(bool optimal, bool is_final, bool has_collisions) {
    // TODO ps what is this created for? kann weg
    std::unique_ptr<VehicleCommandTrajectory>(
            new VehicleCommandTrajectory(get_trajectory_command(t_real_time)));

    Trajectory hlc_comms;
    trajectoryPlan->get_lane_graph_positions(hlc_comms, optimal);
    // Add TimeStamp to each point
    for( unsigned int i=0; i<hlc_comms.lane_graph_positions().size(); i++ ) {
        // Calculate, which point in time this index corresponds to
        uint64_t eta = t_real_time + (trajectoryPlan->get_dt_speed_profile_nanos())*i;
        hlc_comms.lane_graph_positions()[i].estimated_arrival_time().nanoseconds(eta);
    }
    
    if( is_final )
    {
        hlc_comms.type(MessageType::Final);
    }
    else if (optimal)
    {
        hlc_comms.type(MessageType::Optimal);
    } 
    else
    {
        hlc_comms.type(MessageType::Iterative);
    }

    hlc_comms.has_collisions(has_collisions); 
    hlc_comms.vehicle_id(trajectoryPlan->get_vehicle_id());
    hlc_comms.header().create_stamp().nanoseconds(t_real_time);
    hlc_comms.header().valid_after_stamp().nanoseconds(t_real_time + 5000000000ull);

    writer_trajectory->write(hlc_comms);
}

/*
 * Fills in points between two points on the Lane Graph in our other_vehicles_buffer.
 * This naively assumes a fixed speed, equidistant edge_paths-Points
 * and assumes that we are interpolating between two adjacent edges.
 */
void VehicleTrajectoryPlanner::interpolate_vehicles_buffer(
        uint8_t vehicle_id,
        std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer,
        int buffer_index_1, int edge_index_1, int edge_path_index_1,
        int buffer_index_2, int edge_index_2, int edge_path_index_2) 
{

    int delta_edge_path = edge_paths_per_edge - edge_path_index_1 + edge_path_index_2;
    int delta_index = buffer_index_2 - buffer_index_1;
    double step_size = ((double)delta_edge_path)/((double)delta_index);
    
    //std::cout << "sz:" << step_size << "b1:"<< buffer_index_1 << "b2:" << buffer_index_2<<"from: e:"<< edge_index_1 << "ep:" << edge_path_index_1 << "to: e:" << edge_index_2 << "ep:" << edge_path_index_2 <<":";
    // Either start from one past buffer_index_1, or from 0 (buffer_index could be <0)
    for( int i = std::max(buffer_index_1+1, 0); i<buffer_index_2; i++ ) {
        int edge_path_to_insert = edge_path_index_1+int(step_size*round(((double)(i-buffer_index_1+1))));

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

        vehicles_buffer[vehicle_id].insert(vehicles_buffer[vehicle_id].end()-1,
            std::make_pair(i, 
                std::make_pair(
                    edge_to_insert,
                    edge_path_to_insert
                    )
            ));
        //std::cout << "e:" << edge_to_insert << "ep:"<< edge_path_to_insert << ";";
    }
    
}

/*
 * This deletes all points that are in the past from the trajectory point buffer
 * After that, we can just append the new points
 */
// TODO ps naming clear_future_trajectory_point_buffer ?
void VehicleTrajectoryPlanner::clear_past_trajectory_point_buffer() {
    // Delete all trajectory points that are in the future
    // These will be replaced by new points later
    std::vector<TrajectoryPoint>::iterator it_t_now = trajectory_point_buffer.end();
    for (std::vector<TrajectoryPoint>::iterator tp_it = trajectory_point_buffer.begin() + 1;
            tp_it != trajectory_point_buffer.end();
            ++tp_it) {
        if (tp_it->t().nanoseconds() > t_real_time + T_START_DELAY_NANOS
                && it_t_now == trajectory_point_buffer.end()) {
            it_t_now = tp_it-1;
        }
    }

    trajectory_point_buffer.erase(it_t_now, trajectory_point_buffer.end());
}

void VehicleTrajectoryPlanner::set_writer(
        std::unique_ptr< cpm::Writer<Trajectory> > writer){
    writer_trajectory = std::move(writer);
}
void VehicleTrajectoryPlanner::set_reader(
        std::unique_ptr< cpm::ReaderAbstract<Trajectory> > reader){
    reader_trajectory = std::move(reader);
}

void VehicleTrajectoryPlanner::set_fca_writer(
        std::unique_ptr< cpm::Writer<FutureCollisionAssessment> > writer){
    writer_fca = std::move(writer);
}
void VehicleTrajectoryPlanner::set_fca_reader(
        std::unique_ptr< cpm::ReaderAbstract<FutureCollisionAssessment> > reader){
    reader_fca = std::move(reader);
}

/**
 * 
 */
void VehicleTrajectoryPlanner::set_visualization_writer(
    std::unique_ptr< cpm::Writer<Visualization> > writer){
        writer_visualization  = std::move(writer);
}

/**
 * Writes visualization data
 * Priority and FCA 
 */
void VehicleTrajectoryPlanner::display_infos(uint8_t id, std::string info){
    // TODO: implement this in a way that it displays the information smoothly (not lagging behind)
    //std::pair<double, double> pos = trajectoryPlan->get_position();
    //cpm::Logging::Instance().write(1, "write to pos: %lf", trajectory_point_buffer[0].py());

    Visualization vis;
    vis.id(id +100);
    vis.type(VisualizationType::StringMessage);
    //vis.type(VisualizationType::FilledCircle);
    vis.time_to_live(15000000);
    vis.size(0.1);

    std::vector<Point2D> vis_points{ Point2D(trajectory_point_buffer[5].px(), trajectory_point_buffer[5].py()) };
    vis.points(rti::core::vector<Point2D>(vis_points));
    
    Color vis_color(100, 255, 100, 0);
    vis.color(vis_color);
    vis.string_message(info);
    writer_visualization->write(vis);
}


/**
 * 
 */
void VehicleTrajectoryPlanner::write_fca(uint8_t id, uint16_t fca){
    FutureCollisionAssessment fca_message;
    fca_message.vehicle_id(id);
    fca_message.fca(fca);
    fca_message.header().create_stamp().nanoseconds(t_real_time);
    fca_message.header().valid_after_stamp().nanoseconds(t_real_time + 10000ull);
    writer_fca->write(fca_message);
}

/**
 * 
 */
std::vector<uint8_t> VehicleTrajectoryPlanner::compute_priorities(){
    std::lock_guard<std::mutex> lock(collision_assessments_mutex);
    // comparator to sort ascending
    auto cmp = [](std::pair<uint16_t, uint8_t> a, std::pair<uint16_t, uint8_t> b){ return (a.first>b.first || (a.first == b.first && a.second > b.second));};
    // create sortable copy
    std::set<std::pair<uint16_t, uint8_t>, decltype(cmp)> items(cmp);
    for(auto const &kv : collision_assessments){
        items.insert( std::make_pair(kv.second, kv.first));
    }
    // create priority vector with priorities according to the fca
    std::vector<uint8_t> priorities;
    priorities.resize(items.size());
    uint8_t priority = 1;
    for(auto const &vk : items){
        priorities[vk.second - 1] = priority;
        ++priority;
    }
    return priorities;
}

/*
 * Prints contents of other_vehicles_buffer to stdout.
 */
void VehicleTrajectoryPlanner::debug_writeOutReceivedTrajectories() {
    for(auto const& vehicle : other_vehicles_buffer) {
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

std::vector<uint8_t> VehicleTrajectoryPlanner::fca_prio_vec(uint16_t &own_fca) {
    std::set<uint8_t> received_fcas;
    std::set<uint8_t> all_vehicles = coupling_graph.getVehicles();
    std::vector<uint8_t> prio_vec;

    received_fcas.insert(vehicle_id);
    prio_vec.push_back(vehicle_id);

    while( !std::includes(received_fcas.begin(), received_fcas.end(),
                all_vehicles.begin(), all_vehicles.end()) // Becomes true, when we received fca from all active vehicles 
                 && !all_vehicles.empty()
            ) 
    {   
        auto samples = reader_fca->take();
        
        for( FutureCollisionAssessment sample : samples){
            uint64_t t_message = sample.header().create_stamp().nanoseconds();

            if (t_message == t_real_time && sample.vehicle_id() != vehicle_id)
            {
                received_fcas.insert(sample.vehicle_id());
                if (sample.fca() > own_fca || (sample.fca() == own_fca && sample.vehicle_id() < vehicle_id))
                {
                    std::cout << "inserte before";
                    prio_vec.insert(prio_vec.begin(), sample.vehicle_id());
                } 
                else
                {
                    std::cout << "inserted after";
                    prio_vec.push_back(sample.vehicle_id());
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return prio_vec;
}