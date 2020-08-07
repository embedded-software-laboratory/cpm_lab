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

#include "defaults.hpp"

#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_time_ns.hpp"
#include "CommonroadObstacle.hpp"
#include "commonroad_classes/CommonRoadScenario.hpp"

/**
 * \class ObstacleAggregator
 * \brief Keeps received data from commonroad obstacles in map that regards multiple messages + timestamps; analogous to TimeSeriesAggregator but for commonroad obstacles
 *
*/

class ObstacleAggregator
{
    //For visualization of commonroad data - store all received data in the map below, use it to get currently relevant data
    cpm::AsyncReader<CommonroadObstacle> commonroad_obstacle_reader;
    void commonroad_obstacle_receive_callback(dds::sub::LoanedSamples<CommonroadObstacle>& samples);
    std::map<uint8_t, CommonroadObstacle> commonroad_obstacle_data;
    std::mutex commonroad_obstacle_mutex;

    uint64_t reset_time = 0; //After a reset, ignore all previous data (->Header) - remember when last reset was called

public:
    /**
     * \brief The aggregator must be reset not only on start / stop, but also when the scenario changes
     * Thus, a callback is registered at scenario
     * \param scenario CommonRoadScenario - when changed, we need to reset the obstacle aggregator
     */
    ObstacleAggregator(std::shared_ptr<CommonRoadScenario> scenario);
    std::vector<CommonroadObstacle> get_obstacle_data();
    void reset_all_data(); //Reset the data structures if desired by the user (e.g. bc the simulation was stopped)
};