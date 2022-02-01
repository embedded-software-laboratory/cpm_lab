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

#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <exception>
#include <memory>
#include <fstream>
#include "cpm/Logging.hpp"
#include "cpm/Writer.hpp"
#include "cpm/Reader.hpp"
#include "cpm/ReaderAbstract.hpp"
#include "cpm/AsyncReader.hpp"

#include "VehicleCommandTrajectory.hpp"
#include "FutureCollisionAssessment.hpp"
#include "Visualization.hpp"
#include "Trajectory.hpp"
#include "FallbackSync.hpp"

#include "VehicleTrajectoryPlanningState.hpp"
#include "CouplingGraph.hpp"

//using std::vector;


enum class PriorityMode
{
    id,
    random,
    fca,
    vertex_ordering
};

/**
 * \class VehicleTrajectoryPlanner
 * \brief Plans a random trajectory, while communicating planned trajectories with other vehicles.
 * \ingroup decentral_routing
 */
class VehicleTrajectoryPlanner
{
    //! PlanningState plans a route, treating other vehicles as fixed obstacles.
    std::unique_ptr<VehicleTrajectoryPlanningState> trajectoryPlan;
    //! The received trajectories of other vehicles (previous and concurrent) get saved in here; overwritten each timestep
    //std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles_buffer;
    std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> other_vehicles_buffer;
    //! The received trajectories of vehicles of less priority get saved in here
    //std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> ignored_vehicles_buffer;
    std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> ignored_vehicles_buffer;
    //! All trajectories, updated when needed for dyn priorities. Contains optimal predicted and actual planned trajectories.
    std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> vehicles_buffer;
    //! Optimal trajectories of all vehicles
    std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> optimal_vehicles_buffer;
    //! FutureCollisionAssesments of all vehicles; maps vehicle id to its fca
    std::map<uint8_t, uint16_t> collision_assessments;
    //! Mutex to safely access the stored fcas 
    std::mutex collision_assessments_mutex;
    //! Writer to broadcast our own planned or optimal trajectory
    std::unique_ptr< cpm::Writer<Trajectory> > writer_trajectory;
    //! Reader to receive other planned or optimal trajectories
    std::unique_ptr< cpm::ReaderAbstract<Trajectory> > reader_trajectory;
    //! Writer to visualize priority and fca
    std::unique_ptr< cpm::Writer<Visualization> > writer_visualization;
    //! Reader to read FutureCollisionAssessment messages
    std::unique_ptr< cpm::ReaderAbstract<FutureCollisionAssessment> > reader_fca;
    //! Writer to send FutureCollisionAssessments
    std::unique_ptr< cpm::Writer<FutureCollisionAssessment> > writer_fca;
    //! Reader to read FallbackSync messages
    std::unique_ptr< cpm::ReaderAbstract<FallbackSync> > reader_sync;
    //! Writer to send FallbackSync messages
    std::unique_ptr< cpm::Writer<FallbackSync> > writer_sync;
    //! Is false on the first planning step and after the planner crashed
    bool started = false;
    //! Is true, when we either couldn't find any trajectory, or we missed too many timesteps
    bool crashed = false;
    //! Becomes true when we are supposed to stop planning a timestep early
    bool volatile stopFlag = false;
    //! Becomes true, when we are currently not planning a timestep
    bool volatile isStopped = true;
    //! Time in nanoseconds, when we first started planning
    uint64_t t_start = 0;
    //! Time in nanoseconds of the current timestep
    uint64_t t_real_time = 0;
    //! Time in nanoseconds of the previous timestep
    uint64_t t_prev = 0;
    //! Length of the current timestep in nanoseconds
    uint64_t dt_nanos;
    //! Saves trajectory commands we've previously received from PlanningState
    std::vector<TrajectoryPoint> trajectory_point_buffer;
    //! Determines, which vehicles have priority; should be the same for all HLCs
    CouplingGraph coupling_graph;
    //! Counter, of how often in a row we had to abort planning before we were done
    short no_trajectory_counter = 0;
    //! Becomes true when new fcas are present
    bool is_new_fca = true;
    //! The current future collision assessment value of the vehicle
    uint16_t own_fca = 0;
    //! Wether the priorities were updated in comparison to the last call of the plan method
    bool update_priorities = false;
    //! Is true when this vehicle is the winner of the currently active vehicles
    bool token_holder = false;
    //! Is true when the robot hasn't planned it's trajectory yet for this time span
    bool is_active = true;
    //! Marks if the speed_profile should be reset due to new priorities 
    bool reset_speed_profile = false;
    //! Buffer to save a propose new priority assignment 
    std::vector<uint8_t> new_prio_vec;
    //! Old priorities (feasible prios); new_prio_vec is written here when planning with it was successfull
    std::vector<uint8_t> prio_vec;
    //! Set containting the active vehicles still partaking in the distributed priority assignment algorithm of the FCA method.
    std::set<uint8_t> active_vehicles;
    //! vehicle id
    uint8_t vehicle_id;
    //! HLC mode: 0: static prios, 1: random prios, 2: dynamic fca based, 3: vertex ordering
    PriorityMode mode;
    //! Seed for random path generation and random priorities when mode=random
    int seed;
    //! Pseudorandom sequence generator for random priorities
    std::mt19937 random_gen;
    //! Uniform pseudorandom distribution 
    std::uniform_int_distribution<> uniform_distrib;

    // Constants, should be adjusted depending on VehicleTrajectoryPlanningState
    //! How many points we can send in one message; determined by maximum length of vector in DDS
    static constexpr int msg_max_length = 1000;
    //! How many sub-divisions there are per edge on the LaneGraph; determined by geometry.hpp
    static constexpr int edge_paths_per_edge = 25;

    // Evaluation filestream
    std::ofstream evaluation_stream;

    /**
     * \brief Read planned trajectories of vehicles that planned before us
     */
    bool read_previous_vehicles();

    /**
     * \brief Read planned trajectories of vehicles that are planning at the same time as us
     */
    bool read_concurrent_vehicles();

    /**
     * \brief Read planned trajectories of vehicles that are planning after us
     */
    bool wait_for_ignored_vehicles();
    /**
     * \brief Reads the optimal trajectories of all other vehicles
     */
    void read_optimal_trajectories();

    /**
     * \brief Generalized method to read planned trajectories of other vehicles
     * \param vehicle_ids Which vehicle ids should be read (blocks until all these are received)
     * \param write_to_buffer When false, do not write messages to other_vehicles_buffer
     * \param final_messages_only Only read messages with the "final" message attribute
     * \param only_optimal Only read messages containing optimal trajectories
     * \return True if one of the received messages has a collision `false` otherwise.
     */
    bool read_vehicles(std::set<uint8_t> vehicle_ids,
            std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer,
            bool write_to_buffer=false, 
            bool final_messages_only=true,
            bool only_optimal=false);

    /**
     * \brief Method to read the fcas of all active vehicles which haven't yet planned their path this time step
     *          Returns the winner which can plan now and will be removed from the active set
     * \param active_vehicles The vehicles that have to send their fca
     */
    uint8_t get_largest_fca(uint16_t own_fca);

    /**
     * \brief Method to sync after new priorities fail
     * \param active_vehicles The vehicles we will wait for
     * \param feasible bool true iff this vehicle can plan with the new prios; false otherwise
     * \param sync_id uint8_t such that each sync is seperated
     * \return feasible
     */
    bool synchronise(std::set<uint8_t> vehicle_ids, bool feasible, uint8_t sync_id);

    /**
     * \brief Broadcast our planned trajectory to other HLCs
     * \param is_final When false, message is iterative message, else final
     * \param has_collisions Wether this planned trajectory contains collisions
     */
    void send_plan_to_hlcs(bool optimal, bool is_final=true, bool has_collisions=false );

    /**
     * \brief Interpolate between two received trajectory points using fixed velocity
     * \param vehicle_id Which vehicle id these points came from
     * \param vehicles_buffer The buffer to work on
     * \param buffer_index_1 Index of first point in other_vehicle_buffer (might be negative!)
     * \param edge_1 Edge of first point
     * \param edge_index_1 Edge index of first point
     * \param buffer_index_2 Index of second point in other_vehicle_buffer
     * \param edge_2 Edge of second point
     * \param edge_index_2 Edge index of first point
     */
    void interpolate_vehicles_buffer(uint8_t vehicle_id,
            std::map<uint8_t, std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> &vehicles_buffer,
            int buffer_index_1, int edge_1, int edge_index_1,
            int buffer_index_2, int edge_2, int edge_index_2);

    /**
     * \brief Remove all points from the trajectory_point_buffer, that are now in the past
     */
    void clear_past_trajectory_point_buffer();

    /**
     * \brief Create a trajectory command from the newest points in the trajectory_point_buffer
     * \param t_now
     */
    VehicleCommandTrajectory get_trajectory_command(uint64_t t_now);

    /**
     * \brief Debugging method; writes other_vehicles_buffer to stdout
     */
    void debug_writeOutReceivedTrajectories();

    /**
     * \brief Debugging method; writes trajectory_point_buffer to stdout and spots some common issues
     */
    void debug_analyzeTrajectoryPointBuffer();

    /**
     * \brief Debugging method; writes prio_vec to stdout
     **/
    void debug_write_prio_vec();

    /**
     * \brief Computes the priorities of the vehicles based on the current collision_assessments
     * \return std::vector<uint8_t> the priorities of the vehicles (v1, ..., vn) as a vector (pi(v1), ...,pi(vn))
     */
    std::vector<uint8_t> compute_priorities();

    /**
     * \brief Compute and send out own optimal trajectory
     */
    void send_optimal_trajectory();

    /**
     * \brief Computes the prev vehicles based on prio_vec.
     * \return set containing the ids of vehicles with higher priorities.
     **/
    std::set<uint8_t> prev_vehicles();

    /**
     * \brief Ignored vehicles
     * \return set containing the ids of vehicles with lower priorities.
     **/
    std::set<uint8_t> ignored_vehicles();

    /**
    *  \brief Uses the FCA method to plan the vehicles trajectory.
    *  \return True when planning with new priorities was feasbile. False when it was infeasible.
    **/
    bool plan_fca_priorities();

    /**
     * \brief Uses the currently assigned priorities to plan.
     * \return True when planning with current priorities was feasbile. False when it was infeasible.
     */
    bool plan_static_priorities();

    /**
     * \brief Receives the new FCA values of the other vehicles and computes a priority assignment vector.
     * (pi(v1), ..., pi(vn))
     * \return prio_vector containing the vehicle ids in the order of their fca decreasing
     */
    std::vector<uint8_t> fca_prio_vec(uint16_t &own_fca);

    /**
     * \brief Computes a random number which is then used as the FCA and plans using the resulting order.
     * \return True when planning with the new random priorities was feasbile. False when it was infeasible.
     **/
    bool plan_random_priorities();

    /**
     * \brief WIP should plan using the topological sorting and wdfvs approach. 
     * \return True when planning with the new random priorities was feasbile. False when it was infeasible.
     **/
    bool plan_vertex_ordering_priorities();


public:

    /**
     * \brief Create a VehicleTrajectoryPlanner
     */
    VehicleTrajectoryPlanner(const PriorityMode _mode, const int _seed);

    /**
     * \brief Returns started
     */
    bool is_started() {return started;}

    /**
     * \brief Returns crashed
     */
    bool is_crashed() {return crashed;}

    /**
     * \brief Sets the PlanningState of this Planner
     * \param vehicle (cpp-pointer to) PlanningState object
     */
    void set_vehicle(std::unique_ptr<VehicleTrajectoryPlanningState> vehicle);

    /**
     * \brief Sets coupling graph of this Planner; should be identical for all HLCs
     * \param graph CouplingGraph object
     */
    void set_coupling_graph(CouplingGraph graph) {coupling_graph = graph; };

    /**
     * \brief Set writer to send planned trajectories
     * \param writer cpm::Writer object
     */
    void set_writer(std::unique_ptr< cpm::Writer<Trajectory> > writer);

    /**
     * \brief Set reader to read planned trajectories
     * \param reader cpm::ReaderAbstract object
     */
    void set_reader(std::unique_ptr< cpm::ReaderAbstract<Trajectory> > reader);
    
    /**
     * \brief Set reader to read planned trajectories
     * \param reader cpm::ReaderAbstract object
     */
    void set_fca_reader(std::unique_ptr< cpm::ReaderAbstract<FutureCollisionAssessment> > reader);
    
    /**
     * \brief Set writer to send planned trajectories
     * \param writer cpm::Writer object
     */
    void set_fca_writer(std::unique_ptr< cpm::Writer<FutureCollisionAssessment> > writer);

    /**
     * \brief Creates a FutureCollisionAssessment message and publishes it.
     * \param 
     */
    void write_fca(uint8_t id, uint16_t fca, uint8_t iteration);

    /**
     * \brief Sets the visualisation writer
     * \param writer cpm::Writer object
     */
    void set_visualization_writer(std::unique_ptr< cpm::Writer<Visualization> > writer);

    /**
     * \brief Writes out the info to be displayed by the LCC near the vehicle.
     * \param id the id of the vehicle
     * \param info The content that is to be displayed
     */
    void display_infos(uint8_t id, std::string info);

    /**
     * \brief Plan one timestep
     * \param t_real_time Current time in nanoseconds
     * \param dt Length of this timestep
     */
    std::unique_ptr<VehicleCommandTrajectory> plan(uint64_t t_real_time, uint64_t dt);

    /**
     * \brief Stop planning of this timestep, even if we aren't finished
     */
    void stop();
};
