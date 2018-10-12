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


    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicleState, QoS);



    dds::topic::Topic<VehicleCommand> topic_vehicleCommand (participant, "vehicleCommand");
    dds::sub::DataReader<VehicleCommand> reader_vehicleCommand(dds::sub::Subscriber(participant), topic_vehicleCommand);


    VehicleCommand latest_command;
    int latest_command_TTL = 0;
    double position_x = 0;
    double position_y = 0;
    double odometer_meters_prev = 0;


    AbsoluteTimer timer_loop(0, 20000000, 0, 0, [&](){


        {
            vector<dds::sub::Sample<VehicleCommand>> new_commands;
            auto asd = std::back_inserter(new_commands);
            if(reader_vehicleCommand.take(asd) > 0) 
            {
                latest_command = new_commands.back().data();
                latest_command_TTL = 10;

                //std::cout << "thr " << latest_command.motor_throttle() << " str "  << latest_command.steering_angle() << std::endl;
            }
        }


        spi_mosi_data_t spi_mosi_data;
        memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));

        if(latest_command_TTL > 0) {

            double motor_throttle = fmax(-1.0, fmin(1.0, latest_command.motor_throttle()));
            double steering_angle = fmax(-1.0, fmin(1.0, latest_command.steering_angle()));

            uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
            if(motor_throttle > 0.05) motor_mode = SPI_MOTOR_MODE_FORWARD;
            if(motor_throttle < -0.05) motor_mode = SPI_MOTOR_MODE_REVERSE;


            spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
            spi_mosi_data.servo_command = int16_t(steering_angle * 1000.0);
            spi_mosi_data.motor_mode = motor_mode;
            spi_mosi_data.LED_bits = 0b10100110;
        }
        else {
            spi_mosi_data.LED_bits = 0b10101001;
        }



        spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));


        // Exchange data with low level controller
        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);

        uint16_t miso_CRC_actual = spi_miso_data.CRC;
        spi_miso_data.CRC = 0;
        uint16_t mosi_CRC_target = crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));



        const double odometer_meter_per_step = 0.00468384074941;
        const double speed_meter_per_second_per_step = odometer_meter_per_step * 0.2384185791;
        const double imu_yaw_radian_per_step = -0.00109083078249645598;
        const double battery_volt_per_step = 0.0116669;

        if(miso_CRC_actual == mosi_CRC_target) {
            const double yaw_radian = spi_miso_data.imu_yaw * imu_yaw_radian_per_step;
            const double odometer_meters = spi_miso_data.odometer_steps * odometer_meter_per_step;
            const double ds = odometer_meters - odometer_meters_prev;
            odometer_meters_prev = odometer_meters;

            if(-0.5 < ds && ds < 0.5) {
                position_x += ds * cos(yaw_radian);
                position_y += ds * sin(yaw_radian);
            }


            VehicleState sample;
            sample.odometer_distance           (odometer_meters);
            sample.pose().x                    (position_x);
            sample.pose().y                    (position_y);
            sample.pose().yaw                  (yaw_radian);
            sample.imu_acceleration_forward    (spi_miso_data.imu_acceleration_forward * 0.01);
            sample.imu_acceleration_left       (spi_miso_data.imu_acceleration_left * 0.01);
            sample.speed                       (spi_miso_data.speed * speed_meter_per_second_per_step);
            sample.battery_voltage             (spi_miso_data.battery_voltage * battery_volt_per_step);
            sample.motor_current               (spi_miso_data.motor_current); // TODO calibrate current
            writer_vehicleState.write(sample);
        }
        else {
            std::cerr << "[" << std::fixed << std::setprecision(9) << double(clock_gettime_nanoseconds())/1e9 <<  "] Data corruption on ATmega SPI bus. CRC mismatch." << std::endl;
        }


        if(latest_command_TTL > 0) {
            latest_command_TTL--;
        }

    });
    

    while(1) sleep(1);
    return 0;
}
