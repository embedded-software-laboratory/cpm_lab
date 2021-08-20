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
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/get_time_ns.hpp"
#include "Visualization.hpp"

/**
 * \brief This class is used as storage to aggregate all visualization commands received by the LCC (which are drawn in MapViewUi)
 * \ingroup lcc
 */
class VisualizationCommandsAggregator {
private:
    /**
     * \brief Stores new viz messages and potentiall overwrites old ones, depending on their action.
     * Important: User sends time to live, which is converted to a point in time when received 
     * (when that point is reached, the visualization message becomes invalid).
     * \param samples Visualization samples newly received by the async. viz_reader
     */
    void handle_new_viz_msgs(std::vector<Visualization>& samples);

    //! Reader to receive visualization messages sent within the network
    std::shared_ptr<cpm::AsyncReader<Visualization>> viz_reader;
    //! Map to save visualization messages with their ID
    std::map<uint64_t, Visualization> received_viz_map;
    //! Mutex to thread-safely store and access visualization messages in received_viz_map
    std::mutex received_viz_map_mutex;
public:
    /**
     * \brief Constructor, sets up the async visualization message reader viz_reader
     */
    VisualizationCommandsAggregator();

    ~VisualizationCommandsAggregator() {
        std::cout << "!!! --- VisualizationCommandsAggregator destructor" << std::endl;
    }

    /**
     * \brief Returns all viz messages that have been received
     */
    std::vector<Visualization> get_all_visualization_messages();

    /**
     * \brief Resets received_viz_map and thus all visualizations sent so far
     */
    void reset_visualization_commands();
};