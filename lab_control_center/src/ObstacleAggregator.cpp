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

#include "ObstacleAggregator.hpp"

using namespace std::placeholders;

ObstacleAggregator::ObstacleAggregator(std::shared_ptr<CommonRoadScenario> scenario) :
    commonroad_obstacle_reader(
        std::bind(&ObstacleAggregator::commonroad_obstacle_receive_callback, this, _1), 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<CommonroadObstacleList>("commonroadObstacle")
    )
{
    scenario->register_obstacle_aggregator(
        [=] ()
        {
            reset_all_data();
        }
    );
}

void ObstacleAggregator::commonroad_obstacle_receive_callback(dds::sub::LoanedSamples<CommonroadObstacleList>& samples)
{
    std::lock_guard<std::mutex> lock(commonroad_obstacle_mutex);

    for (auto sample : samples) {
        if (sample.info().valid()) {
            //We do not use a reference to the data structure, because we need to make copies at this point (data belongs to the DDS entity)
            for (auto obstacle : sample.data().commonroad_obstacle_list())
            {
                //Ignore if data is older than last reset -> continue to next data point then
                //@Max: Ist das okay so?
                if (obstacle.header().create_stamp().nanoseconds() < reset_time)
                {
                    cpm::Logging::Instance().write(2, "%s", "Received outdated obstacle data (likely event in case of reset)");
                    continue;
                }

                if (commonroad_obstacle_data.find(obstacle.vehicle_id()) != commonroad_obstacle_data.end())
                {
                    //Older obstacle value already exists, keep the newer one - this simple form is sufficient here, create and valid after should be the same
                    if(commonroad_obstacle_data[obstacle.vehicle_id()].header().create_stamp().nanoseconds() < obstacle.header().create_stamp().nanoseconds())
                    {
                        //Store the new obstacle
                        commonroad_obstacle_data[obstacle.vehicle_id()] = obstacle;
                    }
                }
                else
                {
                    //Store the new obstacle
                    commonroad_obstacle_data[obstacle.vehicle_id()] = obstacle;
                }
            }
        }
    }
}

std::vector<CommonroadObstacle> ObstacleAggregator::get_obstacle_data()
{
    std::lock_guard<std::mutex> lock(commonroad_obstacle_mutex);
    std::vector<CommonroadObstacle> return_vec;

    auto current_time = cpm::get_time_ns();

    for (auto entry : commonroad_obstacle_data)
    {
        //Ignore outdated data
        if (entry.second.header().create_stamp().nanoseconds() + timeout < current_time)
        {
            continue;
        }

        return_vec.push_back(entry.second);
    }

    return return_vec;
}

void ObstacleAggregator::reset_all_data()
{
    std::lock_guard<std::mutex> lock(commonroad_obstacle_mutex);
    commonroad_obstacle_data.clear();

    reset_time = cpm::get_time_ns();
}