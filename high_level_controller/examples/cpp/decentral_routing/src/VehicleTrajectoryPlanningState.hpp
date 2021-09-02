#pragma once

#include <vector>
#include <array>
#include <utility>
#include <cstdint>
#include "cpm/Logging.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "HlcCommunication.hpp"

using std::vector;
using std::array;

//TODO: Replace this with a proper variable
#define N_STEPS_SPEED_PROFILE (1000)


/**
 * \class VehicleTrajectoryPlanningState
 * \brief Calculates and saves which trajectory will be driven
 * \ingroup decentral_routing
 */
class VehicleTrajectoryPlanningState
{
    //! Which vehicle id this belongs to
    uint8_t vehicle_id = 0;
    //! Which edge index we are currently at (NOT the current vehicle position)
    size_t current_edge_index = 0;
    //! Which edge path index we are currently at (NOT the current vehicle position)
    size_t current_edge_path_index = 0;
    //! How much beyond the current_edge_path_index we are (in m?)
    double delta_s_path_node_offset = 0;
    //! List of edge indices, which we will plan to next
    vector<size_t> current_route_edge_indices;

    //! How much time has elapsed between when we started planning, and the position we're currently planning for
    uint64_t t_elapsed = 0;

    //! Acceleration used for speeding up/slowing down in m/s^2
    static constexpr double ref_acceleration = 0.8;
    //! Maximum speed in m/s
    static constexpr double max_speed = 1.4;
    //! Minimum speed in m/s
    static constexpr double min_speed = 0.5;
    //! We subdivide each planning steps into time segments this large (ns)
    static constexpr uint64_t dt_speed_profile_nanos = 5*10000000ull;
    //! dt_speed_profile_nanos converted to seconds
    static constexpr double dt_speed_profile = (dt_speed_profile_nanos * 1e-9);
    //! Maximum change in velocity per small timestep
    static constexpr double delta_v_step = ref_acceleration * dt_speed_profile;

    //! Determines, which velocity we need to have at which time; one entry per dt_speed_profile
    array<double, N_STEPS_SPEED_PROFILE> speed_profile;

    /**
     * \brief Sanity check, if we are in a valid state
     */
    void invariant();

    /**
     * \brief Extends current_route_edge_indices with random indices only length n is reached
     * \param n Length that current_route_edge_indices should be extended to
     */
    void extend_random_route(size_t n);

    /**
     * \brief Returns the edge_index/edge_path_index for each speed profile step, in order
     */
    vector<std::pair<size_t, size_t>> get_planned_path();

    /**
     * \brief Set speed at index, and adjust the previous/following speeds as well
     * \param idx_speed_reduction Index of where we want to set the speed
     * \param speed_value Value of speed (m/s)
     */
    void set_speed(int idx_speed_reduction, double speed_value);

public:
    /**
     * \brief Create a new VehicleTrajectoryPlanningState object
     * \param _vehicle_id Which vehicle id the PlanningState belongs to
     * \param _edge_index Which edge index the vehicle is starting on
     * \param _edge_path_index Which edge path index the vehicle is starting on
     */
    VehicleTrajectoryPlanningState(
        uint8_t _vehicle_id,
        size_t _edge_index,
        size_t _edge_path_index
        );

    /**
     * \brief Write out planned trajectory into a HlcCommunication object
     * \param lane_graph_trajectory HlcCommunication object to write into
     */
    void get_lane_graph_positions(HlcCommunication *lane_graph_trajectory);

    /**
     * \brief Returns a single TrajectoryPoint object, with the given parameters
     * \param time Time, when we reach this point (ns)
     * \param edge_index Edge index of point
     * \param edge_path_index Edge path index of point
     * \param speed Which speed we should have at this point
     */
    TrajectoryPoint get_trajectory_point(uint64_t time, size_t edge_index, size_t edge_path_index, double speed);

    /**
     * \brief Returns the currently planned trajectory as a "list" of TrajectoryPoints
     * \param max_length Maximum length of trajectory that should be returned
     * \param dt_nanos How much time each point should be from the next
     */
    vector<TrajectoryPoint> get_planned_trajectory(int max_length, uint64_t dt_nanos);

    /**
     * \brief Advance the planning by a timestep
     * \param dt_nanos Length of timestep in ns
     */
    void apply_timestep(uint64_t dt_nanos);

    /**
     * \brief Return length of one speed profile step
     */
    uint64_t get_dt_speed_profile_nanos() {return dt_speed_profile_nanos;};

    /**
     * \brief Debug method;
     */
    void debug_writeOutOwnTrajectory(); // Debugging method

    /**
     * \brief Change the own speed profile so as not to collide with the other_vehicles.
     * \param other_vehicles
     */
    bool avoid_collisions(std::map<uint8_t, std::map<size_t, std::pair<size_t, size_t>>> other_vehicles);

    /**
     * \brief Returns the corresponding vehicle id
     */
    uint8_t get_vehicle_id(){return vehicle_id;}

};
