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
 * 
 * Note: is_moving is set in the simulation part, because it just depends on the overall trajectory size
 * \ingroup lcc_commonroad
 */
struct ObstacleSimulationSegment
{
    //! (x,y) position of the obstacle. Must be set if lanelet_ref is not set
    std::optional<std::pair<double, double>> position = std::nullopt;
    //! Lanelet reference giving the obstacle's position. Must be set if position is not set
    std::optional<int> lanelet_ref = std::nullopt;
    //! Orientation (yaw) of the obstacle, optional
    std::optional<double> orientation = std::nullopt;
    //! Point in time of the simulation segment for the obstacle. Must exist, but is not default-constructable -> use optional
    std::optional<IntervalOrExact> time = std::nullopt;
    //! Current velocity of the object, obstacle
    std::optional<IntervalOrExact> velocity = std::nullopt;

    //! Occupancy set or lanelet ref might encode current position in form of a shape
    CommonroadDDSShape shape;

    //! If the values stored here (for the position / orientation) are exact or an interval mean
    bool is_exact;
};

/**
 * \struct ObstacleSimulationData
 * \brief This class is used as a return type for the trajectory getter
 * It allows to conveniently store all relevant information of the dynamic obstacle
 * \ingroup lcc_commonroad
 */
struct ObstacleSimulationData
{
    //! List of obstacle data that forms an obstacle's trajectory, with time, position etc.
    std::vector<ObstacleSimulationSegment> trajectory;
    //! Type of obstacle, which does not change over time
    ObstacleType obstacle_type;
};