#pragma once

#include <list>
#include <stdint.h>

#include "VehicleObservation.hpp"
#include "VehicleState.hpp"
#include "SimulationIPS.hpp"
#include <dds/pub/ddspub.hpp>

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

static inline double frand() { return (double(rand()))/RAND_MAX; }

struct ActuatorInput
{
    ActuatorInput();
    ActuatorInput(
        const double& motor_throttle,
        const double& steering_servo,
        const uint8_t& motor_mode)
    :motor_throttle(motor_throttle)
    ,steering_servo(steering_servo)
    ,motor_mode(motor_mode)
    {
    }
    
    double motor_throttle = 0;
    double steering_servo = 0;
    uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
};

class SimulationVehicle
{
    double x = 0.28;//frand()*4;
    double y = 2.05;//frand()*4;
    double distance = 0;
    double yaw = 1.52;//frand()*20;
    double yaw_measured = 1.52;
    double speed = 0;
    double curvature = 0;

    int input_delay = 1;
    // list to simulate time delay
    // last element is the most recent
    std::list<ActuatorInput> actuator_inputs; 

    dds::topic::Topic<VehicleObservation> topic_vehiclePoseSimulated;
    dds::pub::DataWriter<VehicleObservation> writer_vehiclePoseSimulated;

    SimulationIPS& simulationIPS;

public:
    SimulationVehicle(SimulationIPS& _simulationIPS);

    VehicleState update(
        const double& motor_throttle,
        const double& steering_servo,
        const uint8_t& motor_mode,
        const uint64_t& t_now, 
        const double& dt, 
        const uint8_t& vehicle_id
    );

    void get_state(double& _x, double& _y, double& _yaw, double& _speed);
};