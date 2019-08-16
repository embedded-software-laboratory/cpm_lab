#pragma once
#include "defaults.hpp"
#include <dds/pub/ddspub.hpp>
#include "cpm/TimerFD.hpp"
#include "Joystick.hpp"
#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include <functional>

class VehicleManualControl
{
    dds::domain::DomainParticipant& participant;
    shared_ptr<Joystick> joystick = nullptr;
    std::shared_ptr<cpm::TimerFD> update_loop = nullptr;
    uint8_t vehicle_id = 0;
    
    double ref_speed = 0;
    uint64_t ref_trajectory_start_time = 0;
    int ref_trajectory_index = 0;


    dds::topic::Topic<VehicleCommandDirect> topic_vehicleCommandDirect;
    dds::topic::Topic<VehicleCommandSpeedCurvature> topic_vehicleCommandSpeedCurvature;

    shared_ptr<dds::pub::DataWriter<VehicleCommandDirect>> writer_vehicleCommandDirect = nullptr;
    shared_ptr<dds::pub::DataWriter<VehicleCommandSpeedCurvature>> writer_vehicleCommandSpeedCurvature = nullptr;

    std::function<void()> m_update_callback;

public:
    VehicleManualControl();
    void start(uint8_t vehicleId, string joystick_device_file);
    void stop();
    void set_callback(std::function<void()> update_callback) { m_update_callback = update_callback; }
    void get_state(double& throttle, double& steering);
};