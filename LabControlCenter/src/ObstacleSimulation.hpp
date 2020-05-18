#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"

#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "CommonroadObstacle.hpp"
#include <dds/pub/ddspub.hpp>


/**
 * \class ObstacleSimulation
 * \brief Nested class responsible for simulating a single obstacle
 * Objects of this class have access to the writer / other members, e.g. the writer
 */
class ObstacleSimulation
{
private: 
    //DDS
    dds::pub::DataWriter<CommonroadObstacle> writer_commonroad_obstacle;

    //Trajectory info
    uint8_t obstacle_id;
    std::vector<CommonTrajectoryPoint> trajectory; //Important: Position should always be set! Translate lanelet refs beforehand!
    uint64_t time_step_size;
    size_t current_trajectory = 0;

    //Timing
    bool simulated_time;
    std::string node_id;
    uint64_t dt_nanos;
    uint64_t start_time;
    std::shared_ptr<cpm::Timer> timer;

    /**
     * \brief Interpolation function that delivers state values in between set trajectory points
     * \return x,y,yaw values using references as input
     */
    void interpolate_between(CommonTrajectoryPoint p1, CommonTrajectoryPoint p2, double current_time, double &x_interp, double &y_interp, double &yaw_interp);

public:
    /**
     * \brief constructor
     * \param _trajectory The trajectory to follow: Important: Translate lanelet ref to position beforehand, so that it must not be done here anymore (a value is expected for every single trajectory point)
     */
    ObstacleSimulation(std::vector<CommonTrajectoryPoint> _trajectory, double _time_step_size, int _id, bool _simulated_time);

    //Destructor for timer
    ~ObstacleSimulation();

    void start();
    void reset();
};