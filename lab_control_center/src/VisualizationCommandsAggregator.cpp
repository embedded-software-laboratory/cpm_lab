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

        //Set time_to_live to when msg is invalid
        received_viz_map[data.id()].time_to_live(
            cpm::get_time_ns() + data.time_to_live()
        );
    }
}

std::vector<Visualization> VisualizationCommandsAggregator::get_all_visualization_messages() {
    
    std::lock_guard<std::mutex> lock(received_viz_map_mutex);
    uint64_t time_now = cpm::get_time_ns();
    //Delete old viz messages depending on time stamp
    for (std::map<uint64_t, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ) {
        if (it->second.time_to_live() < time_now) {
            it = received_viz_map.erase(it);
        }
        else {
            ++it;
        }
    }

    //Get current viz messages
    std::vector<Visualization> viz_vector;
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