#pragma once

#include "defaults.hpp"

#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_time_ns.hpp"
#include "CommonroadObstacle.hpp"
#include "CommonroadObstacleList.hpp"
#include "commonroad_classes/CommonRoadScenario.hpp"

/**
 * \class ObstacleAggregator
 * \brief Keeps received data from commonroad obstacles in map that regards multiple messages + timestamps; analogous to TimeSeriesAggregator but for commonroad obstacles
 *
*/

class ObstacleAggregator
{
    //For visualization of commonroad data - store all received data in the map below, use it to get currently relevant data
    cpm::AsyncReader<CommonroadObstacleList> commonroad_obstacle_reader;
    void commonroad_obstacle_receive_callback(dds::sub::LoanedSamples<CommonroadObstacleList>& samples);
    std::map<uint8_t, CommonroadObstacle> commonroad_obstacle_data;
    std::mutex commonroad_obstacle_mutex;

    uint64_t reset_time = 0; //After a reset, ignore all previous data (->Header) - remember when last reset was called

public:
    /**
     * \brief The aggregator must be reset not only on start / stop, but also when the scenario changes
     * Thus, a callback is registered at scenario
     * \param scenario CommonRoadScenario - when changed, we need to reset the obstacle aggregator
     */
    ObstacleAggregator(std::shared_ptr<CommonRoadScenario> scenario);
    std::vector<CommonroadObstacle> get_obstacle_data();
    void reset_all_data(); //Reset the data structures if desired by the user (e.g. bc the simulation was stopped)
};