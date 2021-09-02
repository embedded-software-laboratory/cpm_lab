#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include "VehicleCommandTrajectory.hpp"
#include "VehicleTrajectoryPlanningState.hpp"
using std::vector;

/**
 * \class MultiVehicleTrajectoryPlanner
 * \brief TODO
 * \ingroup central_routing
 */
class MultiVehicleTrajectoryPlanner
{
    //! TODO
    std::map<uint8_t, std::shared_ptr<VehicleTrajectoryPlanningState> > trajectoryPlans;
    //! TODO
    bool started = false;
    //! TODO
    uint64_t t_start = 0;
    //! TODO
    uint64_t t_real_time = 0;
    //! TODO
    std::mutex mutex;
    //! TODO
    std::thread planning_thread;
    //! TODO
    const uint64_t dt_nanos;
    //! TODO
    std::map<uint8_t, std::vector<TrajectoryPoint> > trajectory_point_buffer;

public:
    /**
     * \brief Constructor TODO
     * \param dt_nanos TODO
     */
    MultiVehicleTrajectoryPlanner(uint64_t dt_nanos);

    /**
     * \brief Destructor
     */
    ~MultiVehicleTrajectoryPlanner();

    /**
     * \brief TODO
     * \param t_now TODO
     */
    std::vector<VehicleCommandTrajectory> get_trajectory_commands(uint64_t t_now);

    /**
     * \brief TODO
     * \param t TODO
     */
    void set_real_time(uint64_t t);

    /**
     * \brief TODO
     */
    bool is_started() {return started;}

    /**
     * \brief TODO
     * \param vehicle TODO
     */
    void add_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle);

    /**
     * \brief TODO
     */
    void start();

};
