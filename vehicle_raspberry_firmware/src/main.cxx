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
        std::cerr << "Usage: vehicle_rpi_firmware --simulated_time=BOOL --vehicle_id=INT" << std::endl;
        return 1;
    }

    cpm::init(argc, argv);

    const int vehicle_id = cpm::cmd_parameter_int("vehicle_id", 0, argc, argv);
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);


    if(vehicle_id <= 0 || vehicle_id > 25) {
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
        exit(EXIT_FAILURE);
    }
    spi_init();
    const bool allow_simulated_time = false;
#else
    SimulationIPS simulationIPS(topic_vehicleObservation);
    SimulationVehicle simulationVehicle(simulationIPS);
    const bool allow_simulated_time = true;
#endif


	const vector<uint8_t> identification_LED_period_ticks  { 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 };
    const vector<uint8_t> identification_LED_enabled_ticks { 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 };
    
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
    uint64_t t_prev = update_loop->get_time();
    auto log_fn = [&](int line){
        uint64_t now = update_loop->get_time();
        cpm::Logging::Instance().write("PERF LOG L %i T %llu DT %f", line, now, (double(now-t_prev)*1e-6));
        t_prev = now;
    };
    

    // Control loop
    update_loop->start([&](uint64_t t_now) 
    {
        //log_fn(__LINE__);
        try 
        {
            // get IPS observation
            VehicleObservation sample_vehicleObservation;
            uint64_t sample_vehicleObservation_age;
            reader_vehicleObservation.get_sample(t_now, sample_vehicleObservation, sample_vehicleObservation_age);

            spi_mosi_data_t spi_mosi_data;
            memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));

            double motor_throttle = 0;
            double steering_servo = 0;

            //log_fn(__LINE__);

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

            //log_fn(__LINE__);

            // LED identification signal
            {
                spi_mosi_data.LED1_enabled = 1;
                spi_mosi_data.LED2_enabled = 1;
                spi_mosi_data.LED3_enabled = 1;

                if(loop_count % identification_LED_period_ticks.at(vehicle_id) < identification_LED_enabled_ticks.at(vehicle_id))
                {
                    spi_mosi_data.LED4_enabled = 1;
                }
            }

            int n_transmission_attempts = 1;
            int transmission_successful = 1;


#ifdef VEHICLE_SIMULATION
            spi_miso_data_t spi_miso_data = simulationVehicle.update(
                spi_mosi_data, t_now, period_nanoseconds/1e9, vehicle_id);
#else
            // Exchange data with low level micro-controller
            spi_miso_data_t spi_miso_data;

            //auto t_transfer_start = update_loop->get_time();
            spi_transfer(
                spi_mosi_data,
                &spi_miso_data,
                &n_transmission_attempts,
                &transmission_successful
            );
            //auto t_transfer_end = update_loop->get_time();
            //cpm::Logging::Instance().write("spi transfer time %llu, n attempts %i", (t_transfer_end-t_transfer_start), n_transmission_attempts);
#endif

            //log_fn(__LINE__);            

            /*if(n_transmission_attempts > 1)
            {
                cpm::Logging::Instance().write("n_transmission_attempts %i", n_transmission_attempts);
            }*/

            // Process sensor data
            if(transmission_successful) 
            {
                // TODO rethink this. What should be skipped when there is a SPI error?
            
                VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
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
                
                //cpm::Logging::Instance().write("sending state with stamp %llu", vehicleState.header().create_stamp().nanoseconds());
            }
            else 
            {
                cpm::Logging::Instance().write(
                    "Data corruption on ATmega SPI bus. CRC mismatch. After %i attempts.", 
                    n_transmission_attempts);
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
            cpm::Logging::Instance().write("Exception: %s", err_message.c_str());
        }
        
        //log_fn(__LINE__);
    });
    
    return 0;
}
