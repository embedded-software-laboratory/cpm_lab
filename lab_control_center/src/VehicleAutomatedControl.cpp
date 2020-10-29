// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
    task_loop = std::make_shared<cpm::TimerFD>("LCCAutomatedControl", 200000000ull, 0, false);
    
    //Suppress warning for unused parameter in timer (because we only want to show relevant warnings)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"

    task_loop->start_async([&](uint64_t t_now)
    {
        std::lock_guard<std::mutex> lock(stop_list_mutex);
        for (auto iter = vehicle_stop_list.begin(); iter != vehicle_stop_list.end();)
        {
            //Create and send stop signal for the vehicle - TODO, here test value with speed > 0
            VehicleCommandSpeedCurvature stop_command;
            stop_command.vehicle_id(iter->first);
            stop_command.speed(0);
            stop_command.curvature(0);

            cpm::stamp_message(stop_command, cpm::get_time_ns(), 100000000ull);

            writer_vehicleCommandSpeedCurvature->write(stop_command);

            //Delete the vehicle from the map if its message count is zero - then, enough messages should have been sent
            if (iter->second == 0)
            {
                iter = vehicle_stop_list.erase(iter);
            }
            else 
            {
                iter->second = iter->second - 1;
                ++iter;
            }
        }
    },
    [](){
        //Empty lambda callback for stop signals -> Do nothing when a stop signal is received
    });

    #pragma GCC diagnostic pop
}

void VehicleAutomatedControl::stop_vehicles(std::vector<uint8_t> id_list)
{
    //Create and send stop signal for each vehicle
    for (const auto& id : id_list)
    {
        stop_vehicle(id);
        std::cout << "Stopping " << static_cast<int>(id) << std::endl;
    }
}

void VehicleAutomatedControl::stop_vehicle(uint8_t id)
{
    std::lock_guard<std::mutex> lock(stop_list_mutex);
    vehicle_stop_list[id] = 5; //How often the command should be sent
}