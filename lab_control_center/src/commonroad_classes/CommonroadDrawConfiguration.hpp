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

#include <atomic>

/**
 * \struct CommonroadDrawConfiguration
 * \brief This class is not part of the commonroad specs. It defines a data structure that holds information on which
 * parts of the scenario should be visible to the user. The configuration can be set elsewhere and is applied here,
 * in the scenario, when drawing. Some "sub"-classes of the scenario, like lanelet, might have direct access to this
 * file, for easier data sharing
 * We use atomic bool because access from different parts of the program is allowed and desired
 * \ingroup lcc_commonroad
 */
struct CommonroadDrawConfiguration
{
    //! Sets if traffic signs should be drawn in the LCC Map View
    std::atomic_bool draw_traffic_signs{false};
    //! Sets if traffic lights should be drawn in the LCC Map View
    std::atomic_bool draw_traffic_lights{false};
    //! Sets if lanelet type descriptions should be drawn in each lanelet segment in the LCC Map View
    std::atomic_bool draw_lanelet_types{false};
    //! Sets if the lanelet orientation in form of grey arrows should be drawn in the LCC Map View
    std::atomic_bool draw_lanelet_orientation{false};
    //! Sets if goal descriptions should be drawn on the goal states in the LCC Map View
    std::atomic_bool draw_goal_description{false};
    //! Sets if initial state descriptions should be drawn on the goal states in the LCC Map View
    std::atomic_bool draw_init_state{false};
    //! Sets if obstacle descriptions should be drawn on the obstacles in the LCC Map View
    std::atomic_bool draw_obstacle_description{false};
};