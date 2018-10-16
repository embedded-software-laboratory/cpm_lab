#include <iostream>
#include <cstring>
#include <vector>
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

#include "bcm2835.h"

extern "C" {
#include "spi.h"
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.c"
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


int main(/*int argc, char *argv[]*/)
{
    // Hardware setup
    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }
    crcInit();
    spi_init();


    // DDS setup
    dds::domain::DomainParticipant participant (0);

    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicleState, QoS);

    dds::topic::Topic<VehicleCommand> topic_vehicleCommand (participant, "vehicleCommand");
    dds::sub::DataReader<VehicleCommand> reader_vehicleCommand(dds::sub::Subscriber(participant), topic_vehicleCommand);



    // Control loop
    int latest_command_TTL = 0;
    Localization localization;
    Controller controller;

    AbsoluteTimer timer_loop(0, 20000000, 0, 0, [&](){
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
                    latest_command_TTL = 10;
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
        spi_mosi_data_t spi_mosi_data = controller.get_control_signals();


        // Exchange data with low level micro-controller
        spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));
        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
 

        // Process sensor data
        if(check_CRC_miso(spi_miso_data)) {
            VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
            Pose2D new_pose = localization.sensor_update(vehicleState);
            vehicleState.pose(new_pose);
            vehicleState.stamp().nanoseconds(t_iteration_start);

            controller.update_vehicle_state(vehicleState);
            writer_vehicleState.write(vehicleState);


            // temporary monitoring, delete later
            if(vehicleState.battery_voltage() < 6.5) {
                printf("battery_voltage %6.2f V\n", vehicleState.battery_voltage());
                exit(1);
            }
        }
        else {
            std::cerr << "[" << std::fixed << std::setprecision(9) << double(clock_gettime_nanoseconds())/1e9 <<  "] Data corruption on ATmega SPI bus. CRC mismatch." << std::endl;
        }


    });
    

    while(1) sleep(1);
    return 0;
}
