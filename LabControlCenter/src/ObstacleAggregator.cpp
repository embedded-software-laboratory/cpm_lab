#include "ObstacleAggregator.hpp"

using namespace std::placeholders;

ObstacleAggregator::ObstacleAggregator() :
    commonroad_obstacle_reader(
        std::bind(&ObstacleAggregator::commonroad_obstacle_receive_callback, this, _1), 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<CommonroadObstacle>("commonroadObstacle")
    )
{

}

void ObstacleAggregator::commonroad_obstacle_receive_callback(dds::sub::LoanedSamples<CommonroadObstacle>& samples)
{
    std::lock_guard<std::mutex> lock(commonroad_obstacle_mutex);

    for (auto sample : samples) {
        if (sample.info().valid()) {
            CommonroadObstacle obstacle = sample.data();
            if (commonroad_obstacle_data.find(obstacle.vehicle_id()) != commonroad_obstacle_data.end())
            {
                //Older obstacle value already exists, keep the newer one - this simple form is sufficient here, create and valid after should be the same
                if(commonroad_obstacle_data[obstacle.vehicle_id()].header().create_stamp().nanoseconds() < obstacle.header().create_stamp().nanoseconds())
                {
                    //Store the new obstacle
                    commonroad_obstacle_data[obstacle.vehicle_id()] = obstacle;
                }
            }
            else
            {
                //Store the new obstacle
                commonroad_obstacle_data[obstacle.vehicle_id()] = obstacle;
            }
        }
    }
}

std::vector<CommonroadObstacle> ObstacleAggregator::get_obstacle_data()
{
    std::lock_guard<std::mutex> lock(commonroad_obstacle_mutex);
    std::vector<CommonroadObstacle> return_vec;

    for (auto entry : commonroad_obstacle_data)
    {
        return_vec.push_back(entry.second);
    }

    return return_vec;
}

void ObstacleAggregator::reset_all_data()
{
    std::lock_guard<std::mutex> lock(commonroad_obstacle_mutex);
    std::cout << "Reset called - TODO: Pause a bit before clearing, or some messages with delay might get through afterwards and are still drawn" << std::endl;
    commonroad_obstacle_data.clear();
}