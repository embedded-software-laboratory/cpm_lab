#pragma once

#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include <map>
#include <memory>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/get_topic.hpp"

extern "C" {
    #include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

enum class ControllerState
{
    Stop,
    Direct,
    SpeedCurvature,
    Trajectory
};

class Controller
{

    std::unique_ptr< cpm::Reader<VehicleCommandDirect> > reader_CommandDirect;
    std::unique_ptr< cpm::Reader<VehicleCommandSpeedCurvature> > reader_CommandSpeedCurvature;
    std::unique_ptr< cpm::Reader<VehicleCommandTrajectory> > reader_vehicleCommandTrajectory;

    VehicleState m_vehicleState;

    VehicleCommandDirect m_vehicleCommandDirect;
    VehicleCommandSpeedCurvature m_vehicleCommandSpeedCurvature;
    VehicleCommandTrajectory m_vehicleCommandTrajectory;

    ControllerState state = ControllerState::Stop;

    double speed_throttle_error_integral = 0;
    std::map<uint64_t, TrajectoryPoint> trajectory_points;

    double speed_controller(const double speed_measured, const double speed_target);

    void receive_commands(uint64_t t_now);

public:
    Controller(uint8_t vehicle_id);
    void update_vehicle_state(VehicleState vehicleState);
    /*void update_command(VehicleCommandDirect vehicleCommand);
    void update_command(VehicleCommandSpeedCurvature vehicleCommand);
    void update_command(VehicleCommandTrajectory vehicleCommand);
    void vehicle_emergency_stop();*/
    void get_control_signals(uint64_t stamp_now, double &motor_throttle, double &steering_servo);
};