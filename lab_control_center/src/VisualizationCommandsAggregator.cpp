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

#include "VisualizationCommandsAggregator.hpp"

VisualizationCommandsAggregator::VisualizationCommandsAggregator() 
{
    viz_reader = make_shared<cpm::AsyncReader<Visualization>>(
        [this](dds::sub::LoanedSamples<Visualization>& samples){
            handle_new_viz_msgs(samples);
        },
        cpm::ParticipantSingleton::Instance(),
        cpm::get_topic<Visualization>("visualization"),
        true
    );
}

void VisualizationCommandsAggregator::handle_new_viz_msgs(dds::sub::LoanedSamples<Visualization>& samples) {
    std::lock_guard<std::mutex> lock(received_viz_map_mutex);
    for (auto sample : samples) {
        if (sample.info().valid()) {
            //Add new message or replace old one using the id
            received_viz_map[sample.data().id()] = sample.data();

            //Change time stamp from time to live to point in time when msg is invalid
            received_viz_map[sample.data().id()].time_to_live(received_viz_map[sample.data().id()].time_to_live() + cpm::get_time_ns());
        }
    }
}

std::vector<Visualization> VisualizationCommandsAggregator::get_all_visualization_messages() {
    std::vector<Visualization> viz_vector;
    uint64_t time_now = cpm::get_time_ns();
    
    std::lock_guard<std::mutex> lock(received_viz_map_mutex);

    //Delete old viz messages depending on time stamp
    std::vector<uint64_t> delete_ids;
    for (std::map<uint64_t, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ++it) {
        if (it->second.time_to_live() < time_now) {
            delete_ids.push_back(it->first);
        }
    }
    for (uint64_t id : delete_ids) {
        received_viz_map.erase(id);
    }
    
    //Get current viz messages
    for (std::map<uint64_t, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ++it) {
        viz_vector.push_back(it->second);
    }

    return viz_vector;
}

void VisualizationCommandsAggregator::reset_visualization_commands() 
{
    std::lock_guard<std::mutex> lock(received_viz_map_mutex);
    received_viz_map.clear();
}