#include "TypedCommunication.hpp"

/**
 * \file TypedCommunication.cpp
 * \ingroup middleware
 */

template<> void TypedCommunication<VehicleCommandTrajectory>::type_specific_msg_check(VehicleCommandTrajectory msg)
{
    auto set_id = msg.vehicle_id();

    //1. Make sure that enough points have been set (2 points or less are not sufficient)
    if (msg.trajectory_points().size() < 3)
    {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent too few trajectory points, cannot be used for interpolation",
            static_cast<int>(set_id)
        );
    }

    //2. Check how many of the set trajectory points lie in the past / future
    size_t num_past_trajectories = 0;
    auto current_time = cpm::get_time_ns();
    for (auto point : msg.trajectory_points())
    {
        if (point.t().nanoseconds() < current_time)
        {
            ++num_past_trajectories;
        }
        else
        {
            break;
        }
        
    }
    //  a) At least one trajectory point must be in the past, or interpolation is not possible
    if (num_past_trajectories == 0)
    {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent no past trajectory points, cannot be used for interpolation",
            static_cast<int>(set_id)
        );
    }
    //  b) At least one trajectory point must be in the future, or interpolation is not possible
    if (num_past_trajectories == msg.trajectory_points().size())
    {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent no future trajectory points, cannot be used for interpolation",
            static_cast<int>(set_id)
        );
    }
}