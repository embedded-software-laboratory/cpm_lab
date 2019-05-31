#pragma once

#include <stdint.h>
#include "VehicleObservation.hpp"
#include "SimulationIPS.hpp"
#include <dds/pub/ddspub.hpp>

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

static inline double frand() { return (double(rand()))/RAND_MAX; }


class SimulationVehicle
{
    double x = frand()*4;
    double y = frand()*4;
    double distance = 0;
    double yaw = frand()*20;
    double yaw_measured = 0;
    double speed = 0;
    double curvature = 0;

    
    uint32_t tick = 0;
    
    spi_mosi_data_t input_next; // save one input sample to simulate delay time

    dds::topic::Topic<VehicleObservation> topic_vehiclePoseSimulated;
    dds::pub::DataWriter<VehicleObservation> writer_vehiclePoseSimulated;

    SimulationIPS& simulationIPS;

public:
    SimulationVehicle(SimulationIPS& _simulationIPS);
    spi_miso_data_t update(
        const spi_mosi_data_t spi_mosi_data, 
        const uint64_t t_now, 
        const double dt, 
        const uint8_t vehicle_id
    );
    void get_state(double& _x, double& _y, double& _yaw, double& _speed);
};