#pragma once

#include "VehicleCommand.hpp"
#include "VehicleState.hpp"

extern "C" {
    #include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

class Controller
{
    VehicleState m_vehicleState;
    VehicleCommand m_vehicleCommand;
    bool emergency_stop = true;

    double speed_throttle_error_integral = 0;

public:
    void update_vehicle_state(VehicleState vehicleState);
    void update_command(VehicleCommand vehicleCommand);
    void vehicle_emergency_stop();
    spi_mosi_data_t get_control_signals();
};