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
     * Important: User sends time to live, is converted to point in time when received
     */
    void handle_new_viz_msgs(dds::sub::LoanedSamples<Visualization>& samples);

    //Reader to receive samples, map / mutex to thread-safely store and access them
    std::shared_ptr<cpm::AsyncReader<Visualization>> viz_reader;
    std::map<uint64_t, Visualization> received_viz_map;
    std::mutex received_viz_map_mutex;
public:
    VisualizationCommandsAggregator();

    /**
     * \brief Returns all viz messages that have been received
     */
    std::vector<Visualization> get_all_visualization_messages();
};