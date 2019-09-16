#pragma once

#include <list>
#include <stdint.h>

#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/MultiVehicleReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/stamp_message.hpp"
#include "geometry.hpp"
#include "VehicleModel.hpp"
#include "VehicleObservation.hpp"
#include "VehicleState.hpp"
#include "SimulationIPS.hpp"
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

#define INPUT_DELAY 4 // Input delay >= 0
#define MAX_NUM_VEHICLES 30

static inline double frand() { return (double(rand()))/RAND_MAX; }

Pose2D transform_to_COS(const Pose2D origin_COS, const Pose2D pose_in);

class SimulationVehicle
{
    // TODO load parameters via DDS parameters
    std::vector<double> dynamics_parameters = { 1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863 };

    double px;
    double py;
    double distance = 0;
    double yaw;
    double yaw_measured;
    double speed = 0;
    double curvature = 0;

    // array to simulate time delay on inputs
    //      first element is the next to process
    //      last element is received most recently
    double motor_throttle_history[INPUT_DELAY];
    double steering_servo_history[INPUT_DELAY]; 

    dds::topic::Topic<VehicleObservation> topic_vehiclePoseSimulated;
    dds::pub::DataWriter<VehicleObservation> writer_vehiclePoseSimulated;
    cpm::MultiVehicleReader<VehicleObservation> reader_vehiclePoseSimulated;

    SimulationIPS& simulationIPS;

    // For collision checks:
    std::map<uint64_t, Pose2D> ego_pose_history;
    std::map<uint8_t, uint64_t>  get_collisions(const uint64_t t_now, const uint8_t vehicle_id);
    

public:
    SimulationVehicle(SimulationIPS& _simulationIPS, uint8_t vehicle_id);

    VehicleState update(
        const double motor_throttle,
        const double steering_servo,
        const uint64_t t_now, 
        const double dt, 
        const uint8_t vehicle_id
    );

    void get_state(double& _x, double& _y, double& _yaw, double& _speed);
};