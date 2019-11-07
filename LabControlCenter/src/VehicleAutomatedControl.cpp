#include "VehicleAutomatedControl.hpp"

VehicleAutomatedControl::VehicleAutomatedControl() 
:participant(cpm::ParticipantSingleton::Instance())
,topic_vehicleCommandSpeedCurvature(cpm::get_topic<VehicleCommandSpeedCurvature>("vehicleCommandSpeedCurvature"))
{
    //Initialization of the data writer
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    auto publisher = dds::pub::Publisher(participant);
    publisher.default_datawriter_qos(QoS);

    writer_vehicleCommandSpeedCurvature = make_shared<dds::pub::DataWriter<VehicleCommandSpeedCurvature>>(publisher, topic_vehicleCommandSpeedCurvature);
    
    //Initialize the timer (task loop) - here, different tasks like stopping the vehicle are performed
    task_loop = std::make_shared<cpm::TimerFD>("LCCAutomatedControl", 20000000ull, 0, false);
    
    task_loop->start_async([&](uint64_t t_now)
    {
        std::lock_guard<std::mutex> lock(stop_list_mutex);
        for (const auto& id : vehicle_stop_list)
        {
            //Create and send stop signal for the vehicle - TODO, here test value with speed > 0
            VehicleCommandSpeedCurvature stop_command;
            stop_command.vehicle_id(id);
            stop_command.speed(0.5);
            stop_command.curvature(0);

            cpm::stamp_message(stop_command, cpm::get_time_ns(), 1000000000ull);

            writer_vehicleCommandSpeedCurvature->write(stop_command);
        }
    });
}

void VehicleAutomatedControl::stop_vehicles(std::vector<uint8_t> id_list)
{
    //Create and send stop signal for each vehicle
    for (const auto& id : id_list)
    {
        stop_vehicle(id);
    }
}

void VehicleAutomatedControl::stop_vehicle(uint8_t id)
{
    std::lock_guard<std::mutex> lock(stop_list_mutex);
    vehicle_stop_list.push_back(id);
}