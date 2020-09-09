#include "TypedCommunication.hpp"

template<> void TypedCommunication<VehicleCommandTrajectory>::type_specific_msg_check(VehicleCommandTrajectory msg)
{
    //1. Make sure that enough points have been set (2 points or less are not sufficient)
    if (msg.trajectory_points().size() < 3)
    {
        cpm::Logging::Instance().write(
            1,
            "%s",
            "Middleware check failed: HLC script sent too few trajectory points, cannot be used for interpolation"
        );
    }
}