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
#include "CommonroadObstacleList.hpp"
#include "commonroad_classes/CommonRoadScenario.hpp"

/**
 * \class ObstacleAggregator
 * \brief Keeps received data from commonroad obstacles in map that regards multiple messages + timestamps; analogous to TimeSeriesAggregator but for commonroad obstacles; ignores more than 2 seconds old data
 * \ingroup lcc
*/
class ObstacleAggregator
{
    //For visualization of commonroad data - store all received data in the map below, use it to get currently relevant data
    //! Async. reader to receive sent obstacle data
    cpm::AsyncReader<CommonroadObstacleList> commonroad_obstacle_reader;
    /**
     * \brief Callback function for the async reader, to process received obstacle messages
     * \param samples Received obstacle messages
     */
    void commonroad_obstacle_receive_callback(std::vector<CommonroadObstacleList>& samples);
    //! Map that stores obstacle data by ID
    std::map<uint8_t, CommonroadObstacle> commonroad_obstacle_data;
    //! Mutex for accessing commonroad_obstacle_data
    std::mutex commonroad_obstacle_mutex;

    //! Timestamp of last reset. After a reset, ignore all previous data (->Header). Necessary because the loaded scenario / obstacles may change
    uint64_t reset_time = 0;
    //! Ignore all data that is more than two seconds old
    uint64_t timeout = 2e9;

public:
    /**
     * \brief The aggregator must be reset not only on start / stop, but also when the scenario changes
     * Thus, a callback is registered at scenario
     * \param scenario CommonRoadScenario - when changed, we need to reset the obstacle aggregator
     */
    ObstacleAggregator(std::shared_ptr<CommonRoadScenario> scenario);

    ~ObstacleAggregator() {
        std::cout << "!!! --- ObstacleAggregator destructor" << std::endl;
    }

    /**
     * \brief Get all current obstacle data, e.g. for drawing them
     */
    std::vector<CommonroadObstacle> get_obstacle_data();
    
    /**
     * \brief Reset the data structures if desired by the user (e.g. bc the simulation was stopped)
     */
    void reset_all_data(); 
};