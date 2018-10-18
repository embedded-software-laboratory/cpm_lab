#pragma once
#include "defaults.hpp"
#include <dds/pub/ddspub.hpp>
#include "AbsoluteTimer.hpp"
#include "Joystick.hpp"
#include "VehicleCommand.hpp"

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

public:
    VehicleManualControl(shared_ptr<dds::domain::DomainParticipant> participant);
    void start(uint8_t vehicleId, string joystick_device_file);
    void stop();    
};