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

#include <map>
#include <iostream>
#include <vector>
#include "VehicleCommandTrajectory.hpp"

using std::vector;

/**
 * \struct Waypoint
 * \brief TODO
 * \ingroup eight_zero
 */
struct Waypoint
{
    //! TODO
    int index;

    /**
     * \brief The "normal" eight trajectory is extended by one path connecting the two topmost points
     * and by one connecting the two lowermost points. If one of these paths is used the
     * direction in which the vehicle follows the 8-trajectory changes. Thus, all velocities
     * must change the sign which is done by this variable:
     */
    int direction;

    /**
     * \brief Constructor TODO
     * \param index TODO
     * \param direction TODO
     */
    Waypoint(int index, int direction);

    /**
     * \brief TODO
     * \param other TODO
     */
    bool operator<(const Waypoint other) const;
};

/**
 * \class Eight
 * \brief TODO
 * \ingroup eight_zero
 */
class Eight
{
    //! TODO
    std::multimap<Waypoint, Waypoint> next;
    //! TODO
    Waypoint current;
    //! TODO
    Waypoint current2; // it is necessary to plan two points in advice since segment_duration corresponds to the time
                       // in between these two points

    //! needed for get_waypoint because it depends on the chosen way in move_forward
    uint64_t current_segment_duration;
    //! time which the special oval segments need
    uint64_t segment_duration_oval;

    //! TODO
    vector<double> trajectory_px;
    //! TODO
    vector<double> trajectory_py;
    //! TODO
    vector<double> trajectory_vx;
    //! TODO
    vector<double> trajectory_vy;
    //! TODO
    vector<uint64_t> segment_duration;


public:
    /**
     * \brief Constructor TODO
     */
    Eight();
    
    /**
     * \brief TODO
     */
    TrajectoryPoint get_trajectoryPoint();

    /**
     * \brief TODO
     */
    uint64_t get_segment_duration();

    /**
     * \brief TODO
     */
    void move_forward();
};

