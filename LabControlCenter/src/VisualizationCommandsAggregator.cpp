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

    run_thread.store(true);

    viz_time_to_live_thread = std::thread([this] () {
        while(run_thread.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            uint64_t time_now = cpm::get_time_ns();

            std::lock_guard<std::mutex> lock(received_viz_map_mutex);
            std::vector<std::string> delete_ids;
            for (std::map<std::string, Visualization>::iterator it = received_viz_map.begin(); it != received_viz_map.end(); ++it) {
                if (it->second.time_to_live() < time_now) {
                    delete_ids.push_back(it->first);
                }
            }

            for (std::string id : delete_ids) {
                received_viz_map.erase(id);
            }
        }
    });
}

VisualizationCommandsAggregator::~VisualizationCommandsAggregator() {
    run_thread.store(false);
    if (viz_time_to_live_thread.joinable()) {
        viz_time_to_live_thread.join();
    }
}

void VisualizationCommandsAggregator::handle_new_viz_msgs(dds::sub::LoanedSamples<Visualization>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            //Add new message or replace old one using the id
            std::lock_guard<std::mutex> lock(received_viz_map_mutex);
            received_viz_map[sample.data().id()] = sample.data();
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