#include <iostream>
#include <cstring>

#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> // for sleep()

#include "VehicleState.hpp"
#include "AbsoluteTimer.hpp"

#include "bcm2835.h"

extern "C" {
#include "spi.h"
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.c"
}

int main(/*int argc, char *argv[]*/)
{
    crcInit();

    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }
    spi_init();


    dds::domain::DomainParticipant participant (0);
    dds::topic::Topic<VehicleState> topic (participant, "vehicleState");

    auto QoS = dds::pub::qos::DataWriterQos();

    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);


    dds::pub::DataWriter<VehicleState> writer(dds::pub::Publisher(participant), topic, QoS);

    int count = 0;

    AbsoluteTimer timer_loop(0, 20000000, 0, 0, [&](){

        // Exchange data with low level controller
        spi_mosi_data_t spi_mosi_data; // todo receive data, calculate
        memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));

        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);



        uint16_t miso_CRC_actual = spi_miso_data.CRC;
        spi_miso_data.CRC = 0;
        uint16_t mosi_CRC_target = crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));

        if(miso_CRC_actual == mosi_CRC_target) {
            VehicleState sample;
            sample.odometer_distance           (spi_miso_data.odometer_steps);
            sample.imu_yaw                     (spi_miso_data.imu_yaw);
            sample.imu_acceleration_forward    (spi_miso_data.imu_acceleration_forward);
            sample.imu_acceleration_left       (spi_miso_data.imu_acceleration_left);
            sample.speed                       (spi_miso_data.speed);
            sample.battery_voltage             (spi_miso_data.battery_voltage);
            sample.motor_current               (spi_miso_data.motor_current);
            writer.write(sample);
        }
        else {
            std::cerr << "Data corruption on ATmega SPI bus. CRC mismatch." << std::endl;
        }

        count++;

    });
    

    while(1) sleep(1);
    return 0;
}
