#include "TypedCommunication.hpp"

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

template<> void TypedCommunication<VehicleCommandPathTracking>::type_specific_msg_check(VehicleCommandPathTracking msg)
{
    auto set_id = msg.vehicle_id();
    auto path_length = msg.path().size();

    //1. Make sure that enough points have been set (less than 2 points are not sufficient)
    if (path_length < 2) {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent sent too few path tracking points, cannot be used for interpolation",
            static_cast<int>(set_id)
        );
    }

    //2. Make sure the first path tracking point is valid (s must be zero)
    if (msg.path().at(0).s() != 0) {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent invalid first path point, s must be 0",
            static_cast<int>(set_id)
        );
    }

    //3. Make sure consecutive path points have increasing s
    for (std::size_t i = 0; i < path_length - 2; i++)
    {
        std::size_t j = i + 1;
        if (msg.path().at(j).s() <= msg.path().at(i).s()) {
            cpm::Logging::Instance().write(
                1,
                "Middleware (ID %i): HLC script sent invalid path points, s must be increasing",
                static_cast<int>(set_id)
            );
        }
    }

    //4. Make sure first and last pose are identical
    Pose2D first = msg.path().at(0).pose();
    Pose2D last = msg.path().at(path_length - 1).pose();

    if (first.x() != last.x())
    {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent invalid path points, first and last x value differ",
            static_cast<int>(set_id)
        );
    }

    if (first.y() != last.y())
    {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent invalid path points, first and last y value differ",
            static_cast<int>(set_id)
        );
    }

    if (first.yaw() != last.yaw())
    {
        cpm::Logging::Instance().write(
            1,
            "Middleware (ID %i): HLC script sent invalid path points, first and last yaw value differ",
            static_cast<int>(set_id)
        );
    }
}