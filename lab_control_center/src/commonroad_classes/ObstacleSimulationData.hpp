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
    //! Class of obstacle: Dynamic, static or environment
    ObstacleClass obstacle_class;
};