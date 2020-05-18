#pragma once

#include "defaults.hpp"

#include <vector>

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "CommonroadObstacle.hpp"

/**
 * \class ObstacleAggregator
 * \brief Keeps received data from commonroad obstacles in map that regards multiple messages + timestamps; analogous to TimeSeriesAggregator but for commonroad obstacles
 *
*/

class ObstacleAggregator
{
    //For visualization of commonroad data - store all received data in the map below, use it to get currently relevant data
    cpm::AsyncReader<CommonroadObstacle> commonroad_obstacle_reader;
    void commonroad_obstacle_receive_callback(dds::sub::LoanedSamples<CommonroadObstacle>& samples);
    std::map<uint8_t, CommonroadObstacle> commonroad_obstacle_data;
    std::mutex commonroad_obstacle_mutex;

public:
    ObstacleAggregator();
    std::vector<CommonroadObstacle> get_obstacle_data();
    void reset_all_data(); //Reset the data structures if desired by the user (e.g. bc the simulation was stopped)
};