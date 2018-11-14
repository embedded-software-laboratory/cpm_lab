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

#include "VehicleCommand.hpp"
#include "VehicleState.hpp"
#include "AbsoluteTimer.hpp"

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


uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

bool check_CRC_miso(spi_miso_data_t spi_miso_data) { 
    uint16_t mosi_CRC = spi_miso_data.CRC;
    spi_miso_data.CRC = 0;
    return mosi_CRC == crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));
}


int main(int argc, char *argv[])
{

    if(argc != 2) {
        std::cerr << "Usage: vehicle_rpi_firmware <vehicle-id>" << std::endl;
        return 1;
    }

    const int vehicle_id = std::atoi(argv[1]);

    if(vehicle_id < 0 || vehicle_id >= 255) {
        std::cerr << "Invalid vehicle ID." << std::endl;
        return 1;
    }
    std::cout << "vehicle_id " << vehicle_id << std::endl;


#ifndef VEHICLE_SIMULATION
    // Hardware setup
    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }
    spi_init();
#else
    SimulationVehicle simulationVehicle;
    SimulationIPS simulationIPS;
#endif

    crcInit();

    // DDS setup
    dds::domain::DomainParticipant participant (0);

    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicleState, QoS);

    dds::topic::Topic<VehicleCommand> topic_vehicleCommand (participant, "vehicleCommand");
    dds::topic::ContentFilteredTopic<VehicleCommand> topic_vehicleCommand_filtered(topic_vehicleCommand, "vehicleCommand_filtered", dds::topic::Filter("vehicle_id = " + std::to_string(vehicle_id)));
    dds::sub::DataReader<VehicleCommand> reader_vehicleCommand(dds::sub::Subscriber(participant), topic_vehicleCommand_filtered);




    // Control loop
    int latest_command_TTL = 0;
    Localization localization;
    Controller controller;
    int loop_counter = 0;


    const long period_nanoseconds = 20000000; // 50 Hz
    AbsoluteTimer timer_loop(0, period_nanoseconds, 0, 0, [&](){

        try 
        {
            const uint64_t t_iteration_start = clock_gettime_nanoseconds();

            // Read new commands
            {
                vector<dds::sub::Sample<VehicleCommand>> new_commands;
                reader_vehicleCommand.take(std::back_inserter(new_commands));
                for(auto command : new_commands)
                {
                    if(command.info().valid())
                    {
                        controller.update_command(command.data());
                        latest_command_TTL = 25;
                    }
                }
            }

            // Watchdog countdown for loss of signal
            if(latest_command_TTL > 0) {
                latest_command_TTL--;
            }
            else {
                controller.vehicle_emergency_stop();
            }

            // Run controller
            spi_mosi_data_t spi_mosi_data = controller.get_control_signals(t_iteration_start);
            spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));


#ifdef VEHICLE_SIMULATION
            spi_miso_data_t spi_miso_data = simulationVehicle.update(spi_mosi_data, period_nanoseconds/1e9);
#else
            // Exchange data with low level micro-controller
            spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
#endif            
     

            // Process sensor data
            if(check_CRC_miso(spi_miso_data)) {
                VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
                Pose2D new_pose = localization.sensor_update(vehicleState);
                vehicleState.pose(new_pose);
                vehicleState.stamp().nanoseconds(t_iteration_start);
                vehicleState.vehicle_id(vehicle_id);

                controller.update_vehicle_state(vehicleState);
                writer_vehicleState.write(vehicleState);
            }
            else {
                std::cerr << "[" << std::fixed << std::setprecision(9) << double(clock_gettime_nanoseconds())/1e9 <<  "] Data corruption on ATmega SPI bus. CRC mismatch." << std::endl;
            }

            if(loop_counter == 10) {
                localization.reset(); // ignore initial signals
            }

            loop_counter++;
        }
        catch(const dds::core::Exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    });
    

    while(1) sleep(1);
    return 0;
}
