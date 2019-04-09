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

#include "SensorCalibration.hpp"
#include "Localization.hpp"
#include "Controller.hpp"


#ifdef VEHICLE_SIMULATION
#include "SimulationVehicle.hpp"
#include "SimulationIPS.hpp"
#endif

#include "bcm2835.h"

extern "C" {
#include "spi.h"
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h"
}

bool check_CRC_miso(spi_miso_data_t spi_miso_data) { 
    uint16_t mosi_CRC = spi_miso_data.CRC;
    spi_miso_data.CRC = 0;
    return mosi_CRC == crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));
}

template<typename T>
std::unique_ptr<cpm::Reader<T>> make_reader(std::string name, uint8_t vehicle_id)
{
    cpm::VehicleIDFilteredTopic<T> topic(
        dds::topic::Topic<T>(cpm::ParticipantSingleton::Instance(), name), 
        vehicle_id
    );

    return std::unique_ptr<cpm::Reader<T>>(new cpm::Reader<T>(topic));
}

int main(int argc, char *argv[])
{
    if(argc != 2) {
        std::cerr << "Usage: vehicle_rpi_firmware <vehicle-id>" << std::endl;
        return 1;
    }

    const int vehicle_id = std::atoi(argv[1]);

    if(vehicle_id <= 0 || vehicle_id > 25) {
        std::cerr << "Invalid vehicle ID." << std::endl;
        return 1;
    }
    std::cout << "vehicle_id " << vehicle_id << std::endl;


    // DDS setup
    auto& participant = cpm::ParticipantSingleton::Instance();

    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicleState, QoS);

    auto reader_CommandDirect = make_reader<VehicleCommandDirect>("vehicleCommandDirect", vehicle_id);
    auto reader_CommandSpeedCurvature = make_reader<VehicleCommandSpeedCurvature>("vehicleCommandSpeedCurvature", vehicle_id);
    auto reader_vehicleCommandTrajectory = make_reader<VehicleCommandTrajectory>("vehicleCommandTrajectory", vehicle_id);

    dds::topic::Topic<VehicleObservation> topic_vehicleObservation(cpm::ParticipantSingleton::Instance(), "vehicleObservation");
    cpm::VehicleIDFilteredTopic<VehicleObservation> topic_vehicleObservationFiltered(topic_vehicleObservation, vehicle_id);
    cpm::Reader<VehicleObservation> reader_vehicleObservation(topic_vehicleObservationFiltered);



#ifndef VEHICLE_SIMULATION
    // Hardware setup
    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }
    spi_init();
    const bool allow_simulated_time = false;
#else
    SimulationIPS simulationIPS(topic_vehicleObservation);
    SimulationVehicle simulationVehicle(simulationIPS);
    const bool allow_simulated_time = true;
#endif

    crcInit();

	const vector<uint8_t> identification_LED_period_ticks  { 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 };
    const vector<uint8_t> identification_LED_enabled_ticks { 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 };
    
    // Loop setup
    Localization localization;
    Controller controller;
    int loop_count = 0;

    const uint64_t period_nanoseconds = 20000000ull; // 50 Hz
    auto update_loop = cpm::Timer::create("Raspberry_" + std::to_string(vehicle_id), period_nanoseconds, 0, false, allow_simulated_time);

    // Control loop
    update_loop->start([&](uint64_t t_now) {
        try 
        {
            // Read new commands
            {
                VehicleCommandDirect sample_CommandDirect;
                uint64_t sample_CommandDirect_age;
                VehicleCommandSpeedCurvature sample_CommandSpeedCurvature;
                uint64_t sample_CommandSpeedCurvature_age;
                VehicleCommandTrajectory sample_CommandTrajectory;
                uint64_t sample_CommandTrajectory_age;

                reader_CommandDirect->get_sample(t_now, sample_CommandDirect, sample_CommandDirect_age);
                reader_CommandSpeedCurvature->get_sample(t_now, sample_CommandSpeedCurvature, sample_CommandSpeedCurvature_age);
                reader_vehicleCommandTrajectory->get_sample(t_now, sample_CommandTrajectory, sample_CommandTrajectory_age);

                const uint64_t command_timeout = 500000000ull;

                if(    sample_CommandDirect_age         > command_timeout
                    && sample_CommandSpeedCurvature_age > command_timeout
                    && sample_CommandTrajectory_age     > command_timeout)
                {
                    controller.vehicle_emergency_stop();
                }
                else if(sample_CommandDirect_age <= sample_CommandSpeedCurvature_age
                     && sample_CommandDirect_age <= sample_CommandTrajectory_age)
                {
                    controller.update_command(sample_CommandDirect);
                }
                else if(sample_CommandSpeedCurvature_age <= sample_CommandTrajectory_age)
                {
                    controller.update_command(sample_CommandSpeedCurvature);
                }
                else
                {
                    controller.update_command(sample_CommandTrajectory);
                }
            }

            // get IPS observation
            VehicleObservation sample_vehicleObservation;
            uint64_t sample_vehicleObservation_age;
            reader_vehicleObservation.get_sample(t_now, sample_vehicleObservation, sample_vehicleObservation_age);




            spi_mosi_data_t spi_mosi_data;
            memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));

            
            double motor_throttle = 0;
            double steering_servo = 0;

            // Run controller
            {
                controller.get_control_signals(t_now, motor_throttle, steering_servo);

                // Motor deadband, to prevent small stall currents when standing still
                uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
                if(motor_throttle > 0.05) motor_mode = SPI_MOTOR_MODE_FORWARD;
                if(motor_throttle < -0.05) motor_mode = SPI_MOTOR_MODE_REVERSE;

                // Convert to low level controller units
                spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
                spi_mosi_data.servo_command = int16_t(steering_servo * (-1000.0));
                spi_mosi_data.motor_mode = motor_mode;
            }


            // LED identification signal
            {
                spi_mosi_data.LED1_period_ticks = 1;
                spi_mosi_data.LED1_enabled_ticks = 1;
                spi_mosi_data.LED2_period_ticks = 1;
                spi_mosi_data.LED2_enabled_ticks = 1;
                spi_mosi_data.LED3_period_ticks = 1;
                spi_mosi_data.LED3_enabled_ticks = 1;
                spi_mosi_data.LED4_period_ticks = identification_LED_period_ticks.at(vehicle_id);
                spi_mosi_data.LED4_enabled_ticks = identification_LED_enabled_ticks.at(vehicle_id);
            }

            spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));

#ifdef VEHICLE_SIMULATION
            spi_miso_data_t spi_miso_data = simulationVehicle.update(
                spi_mosi_data, t_now, period_nanoseconds/1e9, vehicle_id);
#else
            // Exchange data with low level micro-controller
            spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
#endif            
     

            // Process sensor data
            if(check_CRC_miso(spi_miso_data)) {
                // TODO rethink this. What should be skipped when there is a SPI error?

                VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
                Pose2D new_pose = localization.update(
                    t_now,
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
            else {
                std::cerr << "[" << std::fixed << std::setprecision(9) << double(update_loop->get_time())/1e9 <<  "] Data corruption on ATmega SPI bus. CRC mismatch." << std::endl;
            }


            if(loop_count == 25)
            {
                localization.reset();
            }
            loop_count++;
        }
        catch(const dds::core::Exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    });
    
    return 0;
}
