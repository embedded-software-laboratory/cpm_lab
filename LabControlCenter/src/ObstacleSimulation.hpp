#pragma once

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"

#include <memory>

#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Timer.hpp"
#include "VehicleState.hpp"
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
    dds::pub::DataWriter<VehicleState> writer_vehicleState;

    //Trajectory info
    uint8_t obstacle_id;
    std::vector<CommonTrajectoryPoint> trajectory;
    double time_step_size;
    size_t current_trajectory = 0;

    //Timing
    bool simulated_time;
    std::string node_id;
    uint64_t dt_nanos;
    uint64_t start_time;
    std::shared_ptr<cpm::Timer> timer;

public:
    ObstacleSimulation(std::vector<CommonTrajectoryPoint> _trajectory, double _time_step_size, int _id, bool _simulated_time);

    //Destructor for timer
    ~ObstacleSimulation();

    void start();
    void reset();
};