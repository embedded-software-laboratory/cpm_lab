#pragma once

#include <optional>
#include <vector>

#include "commonroad_classes/datatypes/IntervalOrExact.hpp"

#include "CommonroadDDSShape.hpp"
#include "CommonroadObstacle.hpp"

/**
 * \file ObstacleSimulationData.hpp
 * \brief The structs defined in this class are used by StaticObstacle and DynamicObstacle to summarize all data relevant for 
 * a simulation in one object which can then be easily used by the obstacle simulation
 * \ingroup lcc_commonroad
 */

/**
 * \struct ObstacleSimulationSegment
 * \brief This class is used as a part of the return type for the trajectory getter
 * It allows to conveniently store all relevant information of each CommonTrajectoryPoint
 * \ingroup lcc_commonroad
 */
struct ObstacleSimulationSegment
{
    std::optional<std::pair<double, double>> position = std::nullopt; //x, y
    std::optional<int> lanelet_ref = std::nullopt; //Must be set if position is not set
    std::optional<double> orientation = std::nullopt; //yaw
    std::optional<IntervalOrExact> time = std::nullopt; //Must exist, but is not default-constructable -> use optional
    std::optional<IntervalOrExact> velocity = std::nullopt;

    CommonroadDDSShape shape; //Occupancy set or lanelet ref might encode current position in form of a shape

    bool is_exact;
    //is_moving is set in the simulation part, because it just depends on the overall trajectory size
};

/**
 * \struct ObstacleSimulationData
 * \brief This class is used as a return type for the trajectory getter
 * It allows to conveniently store all relevant information of the dynamic obstacle
 * \ingroup lcc_commonroad
 */
struct ObstacleSimulationData
{
    std::vector<ObstacleSimulationSegment> trajectory;
    ObstacleType obstacle_type;
};