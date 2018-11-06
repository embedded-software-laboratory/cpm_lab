#pragma once
#include "defaults.hpp"
#include <dds/pub/ddspub.hpp>
#include "AbsoluteTimer.hpp"
#include "Joystick.hpp"
#include "VehicleCommand.hpp"
#include <functional>

class VehicleManualControl
{
    shared_ptr<dds::domain::DomainParticipant> participant;
    shared_ptr<Joystick> joystick = nullptr;
    shared_ptr<AbsoluteTimer> update_loop = nullptr;
    uint8_t vehicle_id = 0;
    
    double ref_speed = 0;
    uint64_t ref_trajectory_start_time = 0;
    int ref_trajectory_index = 0;


    shared_ptr<dds::topic::Topic<VehicleCommand>> topic_vehicleCommand = nullptr;
    shared_ptr<dds::pub::DataWriter<VehicleCommand>> writer_vehicleCommand = nullptr;

    std::function<void()> m_update_callback;

public:
    VehicleManualControl(shared_ptr<dds::domain::DomainParticipant> participant);
    void start(uint8_t vehicleId, string joystick_device_file);
    void stop();
    void set_callback(std::function<void()> update_callback) { m_update_callback = update_callback; }
    void get_state(double& throttle, double& steering);
};