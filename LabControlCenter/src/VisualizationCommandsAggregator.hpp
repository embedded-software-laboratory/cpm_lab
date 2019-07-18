#pragma once

#include <mutex>
#include <vector>

#include "Visualization.hpp"

/**
 * \brief This class is used as storage to aggregate all visualization commands received by the LCC (which are drawn in MapViewUi)
 */

class VisualizationCommandsAggregator {
private:
    std::map<long, Visualization> received_viz_map;
    std::mutex received_viz_map_mutex;
public:
    VisualizationCommandsAggregator();

    std::vector<Visualization> get_all_visualization_messages();
};