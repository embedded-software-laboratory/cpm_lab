#pragma once

#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include <map>
#include <memory>
#include <mutex>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "MpcController.hpp"
#include "TrajectoryInterpolation.hpp"

extern "C" {
    #include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
}

#define TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE 1500

enum class ControllerState
{
    Stop,
    Direct,
    SpeedCurvature,
    Trajectory
};

class Controller
{
    MpcController mpcController;

    std::function<uint64_t()> m_get_time;

    std::unique_ptr< cpm::Reader<VehicleCommandDirect> > reader_CommandDirect;
    std::unique_ptr< cpm::Reader<VehicleCommandSpeedCurvature> > reader_CommandSpeedCurvature;

    std::unique_ptr< cpm::AsyncReader<VehicleCommandTrajectory> > asyncReader_vehicleCommandTrajectory;

    VehicleState m_vehicleState;

    VehicleCommandDirect m_vehicleCommandDirect;
    VehicleCommandSpeedCurvature m_vehicleCommandSpeedCurvature;
    

    ControllerState state = ControllerState::Stop;

    const uint64_t command_timeout = 500000000ull;

    uint64_t latest_command_receive_time = 0;

    double speed_throttle_error_integral = 0;

    std::map<uint64_t, TrajectoryPoint> m_trajectory_points;
    std::mutex command_receive_mutex;


    // TODO remove linear trajectory controller related stuff, once the MPC works well
    double trajectory_controller_lateral_P_gain;
    double trajectory_controller_lateral_D_gain;
    void trajectory_controller_linear(uint64_t t_now, double &motor_throttle_out, double &steering_servo_out);

    void update_remote_parameters();

    double speed_controller(const double speed_measured, const double speed_target);


    void receive_commands(uint64_t t_now);

    void reveice_trajectory_callback(dds::sub::LoanedSamples<VehicleCommandTrajectory>& samples);
    std::shared_ptr<TrajectoryInterpolation> interpolate_trajectory_command(uint64_t t_now);

    // Trajectory tacking statistics
    void trajectory_tracking_statistics_update(uint64_t t_now);
    double trajectory_tracking_statistics_longitudinal_errors  [TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE];
    double trajectory_tracking_statistics_lateral_errors       [TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE];
    size_t trajectory_tracking_statistics_index = 0;

public:
    Controller(uint8_t vehicle_id, std::function<uint64_t()> _get_time);
    void update_vehicle_state(VehicleState vehicleState);
    void get_control_signals(uint64_t stamp_now, double &motor_throttle, double &steering_servo);
};