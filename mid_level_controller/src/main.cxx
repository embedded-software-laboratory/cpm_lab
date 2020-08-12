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

#include <atomic>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <string>
#include <iterator>
#include <iomanip>
#include <ios>
using std::vector;

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/config/Logger.hpp>
#include <rti/util/util.hpp> // for sleep()

#include "VehicleObservation.hpp"
#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include "cpm/Timer.hpp"
#include "cpm/VehicleIDFilteredTopic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Reader.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"

#include "SensorCalibration.hpp"
#include "Localization.hpp"
#include "Controller.hpp"


#ifdef VEHICLE_SIMULATION
#include "SimulationVehicle.hpp"
#include "SimulationIPS.hpp"
#endif

#include "bcm2835.h"


int main(int argc, char *argv[])
{
    //rti::config::Logger::instance().verbosity(rti::config::Verbosity::STATUS_ALL);
    //rti::config::Logger::instance().verbosity(rti::config::Verbosity::WARNING);

    if(argc < 2) {
        std::cerr << "Usage: vehicle_rpi_firmware --simulated_time=BOOL --vehicle_id=INT --dds_domain=INT(optional) --pose=DOUBLE,DOUBLE,DOUBLE(optional;only simulation; x,y,yaw)" << std::endl;
        return 1;
    }

    cpm::init(argc, argv);

    const int vehicle_id = cpm::cmd_parameter_int("vehicle_id", 0, argc, argv);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    if(vehicle_id <= 0 || vehicle_id > 255) { //Upper bound due to use of uint8_t
        std::cerr << "Invalid vehicle ID." << std::endl;
        return 1;
    }
    std::cout << "vehicle_id " << vehicle_id << std::endl;
    cpm::Logging::Instance().set_id("vehicle_raspberry_" + std::to_string(vehicle_id));

    // DDS setup
    auto& participant = cpm::ParticipantSingleton::Instance();

    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");

    dds::pub::DataWriter<VehicleState> writer_vehicleState(
        dds::pub::Publisher(participant), 
        topic_vehicleState, 
        dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::BestEffort()
    );

    dds::topic::Topic<VehicleObservation> topic_vehicleObservation(cpm::ParticipantSingleton::Instance(), "vehicleObservation");
    cpm::VehicleIDFilteredTopic<VehicleObservation> topic_vehicleObservationFiltered(topic_vehicleObservation, vehicle_id);
    cpm::Reader<VehicleObservation> reader_vehicleObservation(topic_vehicleObservationFiltered);

#ifndef VEHICLE_SIMULATION
    // Hardware setup
    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        cpm::Logging::Instance().write(1, "%s", "bcm2835_init failed. Are you running as root??");
        exit(EXIT_FAILURE);
    }
    spi_init();
    const bool allow_simulated_time = false;
#else
    // Get vehicle starting position from argv
    vector<double> starting_position = cpm::cmd_parameter_doubles("pose", {0.0}, argc, argv);
    if(starting_position.size() != 1 && starting_position.size() != 3) {
        starting_position = {0.0};
    }
    SimulationIPS simulationIPS(topic_vehicleObservation);
    SimulationVehicle simulationVehicle(simulationIPS, vehicle_id, starting_position);
    const bool allow_simulated_time = true;
#endif

    // Loop setup
    int64_t loop_count = 0;

    const uint64_t period_nanoseconds = 20000000ull; // 50 Hz
    auto update_loop = cpm::Timer::create(
        "vehicle_raspberry_" + std::to_string(vehicle_id), 
        period_nanoseconds, 0, 
        false, 
        allow_simulated_time,
        enable_simulated_time);

    Localization localization;
    Controller controller(vehicle_id, [&](){return update_loop->get_time();});

    
    // Timing / profiling helper
    // uint64_t t_prev = update_loop->get_time();
    // auto log_fn = [&](int line){
    //     uint64_t now = update_loop->get_time();
    //     cpm::Logging::Instance().write("PERF LOG L %i T %llu DT %f", line, now, (double(now-t_prev)*1e-6));
    //     t_prev = now;
    // };
    
    //----------------------------------------------------------------------------------------------------------------------

    //Variabel for the control loop: If a stop signal was received, reset the controller, stop the vehicle, wait, then restart after a while
    //After a while: Wait to make sure that old messages are ignored
    unsigned int STOP_STEPS = 50; //50Hz -> pause for one second
    std::atomic_uint_least32_t stop_counter; //Reset control_stop to false again after some iterations, so that the controller is not reset immediately (ignore old messages) - sleep would lead to problems regarding the VehicleObservation
    stop_counter.store(0); //0 means normal run

    // Control loop
    update_loop->start(
        //Callback for update signal
        [&](uint64_t t_now) 
        {
            //Log control cycle period
            //For evaluation log of vehicle cycle period
            cpm::Logging::Instance().write(3, "Control cycle timestamp: %llu", update_loop->get_time());

            //log_fn(__LINE__);
            try 
            {
                // get IPS observation
                VehicleObservation sample_vehicleObservation;
                uint64_t sample_vehicleObservation_age;
                reader_vehicleObservation.get_sample(
                    t_now,
                    sample_vehicleObservation,
                    sample_vehicleObservation_age
                );

                double motor_throttle = 0;
                double steering_servo = 0;

                // Run controller only if no stop signal was received, else do not drive
                //The controller gets reset at the end of the function, to make sure that before that the vehicle actually gets to stop driving
                if(stop_counter.load() == 0)
                {
                    controller.get_control_signals(t_now, motor_throttle, steering_servo);
                }
                else 
                {
                    controller.get_stop_signals(motor_throttle, steering_servo);
                }

                int n_transmission_attempts = 1;
                int transmission_successful = 1;


    #ifdef VEHICLE_SIMULATION
                VehicleState vehicleState = simulationVehicle.update(
                    motor_throttle,
                    steering_servo,
                    t_now,
                    period_nanoseconds/1e9,
                    vehicle_id
                );
                vehicleState.is_real(false); // Is not real, is simulated
    #else
                // Motor deadband, to prevent small stall currents when standing still
                uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
                if(motor_throttle > 0.05) motor_mode = SPI_MOTOR_MODE_FORWARD;
                if(motor_throttle < -0.05) motor_mode = SPI_MOTOR_MODE_REVERSE;

                spi_mosi_data_t spi_mosi_data;
                memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
                
                // Convert actuator input to low level controller units
                spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
                spi_mosi_data.servo_command = int16_t(steering_servo * (-1000.0));
                spi_mosi_data.motor_mode = motor_mode;

                // vehicle ID for LED flashing 
                spi_mosi_data.vehicle_id = static_cast<uint8_t>(vehicle_id);

                // Exchange data with low level micro-controller
                spi_miso_data_t spi_miso_data;

                //auto t_transfer_start = update_loop->get_time();
                spi_transfer(
                    spi_mosi_data,
                    &spi_miso_data,
                    &n_transmission_attempts,
                    &transmission_successful
                );

                VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
                vehicleState.is_real(true); // Is real, is not simulated
    #endif

                // Process sensor data
                if(transmission_successful) 
                {
                    Pose2D new_pose = localization.update(
                        t_now,
                        period_nanoseconds,
                        vehicleState,
                        sample_vehicleObservation, 
                        sample_vehicleObservation_age
                    );
                    vehicleState.pose(new_pose);
                    vehicleState.motor_throttle(motor_throttle);
                    vehicleState.steering_servo(steering_servo);
                    vehicleState.vehicle_id(vehicle_id);
                    vehicleState.IPS_update_age_nanoseconds(sample_vehicleObservation_age);
                    cpm::stamp_message(vehicleState, t_now, 60000000ull);

                    controller.update_vehicle_state(vehicleState);
                    writer_vehicleState.write(vehicleState);
                }
                else 
                {
                    cpm::Logging::Instance().write(
                        1,
                        "Data corruption on ATmega SPI bus. CRC mismatch. After %i attempts.", 
                        n_transmission_attempts
                    );
                }

                if(loop_count == 25)
                {
                    localization.reset();
                }
                loop_count++;
            }
            catch(const dds::core::Exception& e)
            {
                //std::cerr << "Exception: " << e.what() << std::endl;
                std::string err_message = e.what();
                cpm::Logging::Instance().write(
                    1,
                    "Error: %s",
                    err_message.c_str());
            }

            //If a stop signal was received, stop the vehicle (was done above, get_control_signals is ignored)
            //Then, wait one second and reset the controller to make sure that old data gets ignored for future commands
            if (stop_counter.load() > 0)
            {
                //Reset if the counter will stop in the next round
                if (stop_counter.load() == 1)
                {
                    controller.reset();
                }

                //Decrement the counter
                stop_counter.store(stop_counter.load() - 1);
            }
            
            //log_fn(__LINE__);
        },
    //Callback for stop signal
        [&](){
            //Clear all recent commands and make the vehicle stop immediately, and prevent receiving new data for a limited amount of time
            //Define x empty runs before the reset
            stop_counter.store(STOP_STEPS); //50Hz -> pause for one second

            //Use %s, else we get a warning that this is no string literal (we do not want unnecessary warnings to show up)
            cpm::Logging::Instance().write(
                3,
                "Received stop %s",
                "signal");
        }
    );
    
    return 0;
}
