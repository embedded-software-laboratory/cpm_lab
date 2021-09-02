#include "VisualizationCommandsAggregator.hpp"

/**
 * \file VisualizationCommandsAggregator.cpp
 * \ingroup lcc
 */

VisualizationCommandsAggregator::VisualizationCommandsAggregator() 
{
    viz_reader = make_shared<cpm::AsyncReader<Visualization>>(
        [this](std::vector<Visualization>& samples){
            handle_new_viz_msgs(samples);
        }
        ,"visualization"
    );
}

void VisualizationCommandsAggregator::handle_new_viz_msgs(std::vector<Visualization>& samples) {
    std::lock_guard<std::mutex> lock(received_viz_map_mutex);
    for (auto& data : samples) {
        //Add new message or replace old one using the id
        received_viz_map[data.id()] = data;

        //Change time stamp from time to live to point in time when msg is invalid
        received_viz_map[data.id()].time_to_live(received_viz_map[data.id()].time_to_live() + cpm::get_time_ns());
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