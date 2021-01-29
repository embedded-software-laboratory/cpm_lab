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

#include "Localization.hpp"
#include <cmath>
#include <iostream>

/**
 * \file Localization.cxx
 * \ingroup vehicle
 */

/**
 * \brief TODO
 * \param previous TODO
 * \param current TODO
 * \ingroup vehicle
 */
void filter_update_step(const LocalizationState& previous, LocalizationState& current)
{
    Pose2D new_pose = previous.pose;

    // Dead reckoning update
    {
        double delta_s = current.odometer_distance - previous.odometer_distance;
        double delta_yaw = remainder(current.imu_yaw - previous.imu_yaw, 2*M_PI);

        // ignore signal discontinuities
        if(!(-0.5 < delta_s && delta_s < 0.5)) {
            delta_s = 0;
        }

        if(!(-1.5 < delta_yaw && delta_yaw < 1.5)) {
            delta_yaw = 0;
        }

        new_pose.yaw(new_pose.yaw() + delta_yaw);
        new_pose.x(new_pose.x() + delta_s * cos(new_pose.yaw()));
        new_pose.y(new_pose.y() + delta_s * sin(new_pose.yaw()));
    }

    // IPS update
    if(current.has_valid_observation)
    {
        // TODO proper kalman filter

        const double s = 0.3;

        double dx = current.vehicleObservation.pose().x() - new_pose.x();
        double dy = current.vehicleObservation.pose().y() - new_pose.y();
        double dyaw = current.vehicleObservation.pose().yaw() - new_pose.yaw();
        dyaw = remainder(dyaw, 2*M_PI);

        new_pose.x(new_pose.x() + s*dx);
        new_pose.y(new_pose.y() + s*dy);
        new_pose.yaw(new_pose.yaw() + s*dyaw);
    }


    new_pose.yaw(remainder(new_pose.yaw(), 2*M_PI));
    current.pose = new_pose;
}


Pose2D Localization::update(
    uint64_t t_now,
    uint64_t period,
    VehicleState vehicleState,
    VehicleObservation sample_vehicleObservation,
    uint64_t sample_vehicleObservation_age
)
{

    // Save new sensor data, update current step
    {
        LocalizationState localizationStateNew;
        localizationStateNew.t = t_now;
        localizationStateNew.imu_yaw = vehicleState.imu_yaw();
        localizationStateNew.odometer_distance = vehicleState.odometer_distance();
        write_next_state(localizationStateNew);

        filter_update_step(get_state(LOCALIZATION_BUFFER_SIZE-2), get_state(LOCALIZATION_BUFFER_SIZE-1));
    }

    // Check for new observation. Reprocess if necessary
    if(sample_vehicleObservation_age < 10000000000ull)
    {
        int reprocessing_start_index = -1;

        // search for state index corresponding to the given observation
        for (int i = LOCALIZATION_BUFFER_SIZE-1; i > 0; i--)
        {
            LocalizationState& state_i = get_state(i);

            // found the corresponding state
            if(state_i.t <= sample_vehicleObservation.header().create_stamp().nanoseconds()
                && sample_vehicleObservation.header().create_stamp().nanoseconds() < (state_i.t + period))
            {
                // have we already done this on a previous update()? then skip it
                if(state_i.has_valid_observation) break;

                state_i.has_valid_observation = true;
                state_i.vehicleObservation = sample_vehicleObservation;

                // we have a new observation, need to do reprocessing
                reprocessing_start_index = i;
                break;
            }
        }

        // reprocessing
        if(reprocessing_start_index > 0)
        {
            for (int i = reprocessing_start_index; i < LOCALIZATION_BUFFER_SIZE; ++i)
            {
                filter_update_step(get_state(i-1), get_state(i));
            }
        }
    }


    // output latest pose
    return get_state(LOCALIZATION_BUFFER_SIZE-1).pose;
}

void Localization::reset() {
    for (size_t i = 0; i < LOCALIZATION_BUFFER_SIZE; ++i)
    {
        state_buffer[i].pose = Pose2D(0,0,0);
    }
}