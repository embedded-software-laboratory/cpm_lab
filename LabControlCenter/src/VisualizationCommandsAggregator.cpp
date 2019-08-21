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
    std::vector<std::string> delete_ids;
    for (std::map<std::string, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ++it) {
        if (it->second.time_to_live() < time_now) {
            delete_ids.push_back(it->first);
        }
    }
    for (std::string id : delete_ids) {
        received_viz_map.erase(id);
    }
    
    //Get current viz messages
    for (std::map<std::string, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ++it) {
        viz_vector.push_back(it->second);
    }

    return viz_vector;
}