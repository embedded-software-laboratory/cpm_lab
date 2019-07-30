#pragma once

#include "defaults.hpp"
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
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
public:
    VisualizationCommandsAggregator();

    /**
     * \brief Returns all viz messages that have been received
     */
    std::vector<Visualization> get_all_visualization_messages();
};