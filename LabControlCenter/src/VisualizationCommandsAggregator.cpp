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
    for (auto sample : samples) {
        if (sample.info().valid()) {
            //Add new message or delete old one using the id
            if (sample.data().action() == VisualizationAction::ADD) {
                std::lock_guard<std::mutex> lock(received_viz_map_mutex);

                received_viz_map[sample.data().id()] = sample.data();
            }
            else {
                std::lock_guard<std::mutex> lock(received_viz_map_mutex);

                received_viz_map.erase(sample.data().id());
            }
        }
    }
}

std::vector<Visualization> VisualizationCommandsAggregator::get_all_visualization_messages() {
    std::vector<Visualization> viz_vector;
    
    std::lock_guard<std::mutex> lock(received_viz_map_mutex);
    for (std::map<std::string, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ++it) {
        viz_vector.push_back(it->second);
    }

    return viz_vector;
}