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
 */

class VisualizationCommandsAggregator {
private:
    /**
     * \brief Stores new viz messages or deletes old ones, depending on their action
     */
    void handle_new_viz_msgs(dds::sub::LoanedSamples<Visualization>& samples);

    //Reader to receive samples, map / mutex to thread-safely store and access them
    std::shared_ptr<cpm::AsyncReader<Visualization>> viz_reader;
    std::map<std::string, Visualization> received_viz_map;
    std::mutex received_viz_map_mutex;

    //Thread that checks if messages have timed out
    std::thread viz_time_to_live_thread;
    std::atomic_bool run_thread;
public:
    VisualizationCommandsAggregator();
    ~VisualizationCommandsAggregator();

    /**
     * \brief Returns all viz messages that have been received
     */
    std::vector<Visualization> get_all_visualization_messages();
};