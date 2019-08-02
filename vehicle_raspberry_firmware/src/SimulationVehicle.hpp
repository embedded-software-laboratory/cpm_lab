#pragma once

#include <list>
#include <stdint.h>

#include "cpm/AsyncReader.hpp"
#include "VehicleModel.hpp"
#include "VehicleObservation.hpp"
#include "VehicleState.hpp"
#include "SimulationIPS.hpp"
#include <dds/pub/ddspub.hpp>

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

#define INPUT_DELAY (1) // Input delay >= 0

static inline double frand() { return (double(rand()))/RAND_MAX; }

class SimulationVehicle
{
    // TODO load parameters via DDS parameters
    std::vector<double> dynamics_parameters = { 1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863 };

    double px = 0.28;//frand()*4;
    double py = 2.05;//frand()*4;
    double distance = 0;
    double yaw = 1.52;//frand()*20;
    double yaw_measured = 1.52;
    double speed = 0;
    double curvature = 0;

    // array to simulate time delay on inputs
    //      first element is the next to process
    //      last element is received most recently
    double motor_throttle_history[INPUT_DELAY];
    double steering_servo_history[INPUT_DELAY]; 

    dds::topic::Topic<VehicleObservation> topic_vehiclePoseSimulated;
    dds::pub::DataWriter<VehicleObservation> writer_vehiclePoseSimulated;

    SimulationIPS& simulationIPS;

    // For collision checks:
    std::map<uint8_t, Pose2D> vehiclesPoses;
    void check_for_collisions();
    void receive_vehicle_observation_callback(
        dds::sub::LoanedSamples<VehicleObservation>& samples
    );
    cpm::AsyncReader<VehicleObservation> reader_vehicle_observation;


public:
    SimulationVehicle(SimulationIPS& _simulationIPS);

    VehicleState update(
        const double& motor_throttle,
        const double& steering_servo,
        const uint64_t& t_now, 
        const double& dt, 
        const uint8_t& vehicle_id
    );

    void get_state(double& _x, double& _y, double& _yaw, double& _speed);
};