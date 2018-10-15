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
    Localization localization;


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


            switch(latest_command.data()._d().underlying()) {
                
                case VehicleCommandMode::DirectControlMode:
                {
                    double motor_throttle = fmax(-1.0, fmin(1.0, latest_command.data().direct_control().motor_throttle()));
                    double steering_angle = fmax(-1.0, fmin(1.0, latest_command.data().direct_control().steering_angle()));

                    uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
                    if(motor_throttle > 0.05) motor_mode = SPI_MOTOR_MODE_FORWARD;
                    if(motor_throttle < -0.05) motor_mode = SPI_MOTOR_MODE_REVERSE;


                    spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
                    spi_mosi_data.servo_command = int16_t(steering_angle * 1000.0);
                    spi_mosi_data.motor_mode = motor_mode;
                    spi_mosi_data.LED_bits = 0b10100110;
                }
                break;

                case VehicleCommandMode::SpeedCurvatureMode:
                {
                    // TODO
                    std::cerr << "SpeedCurvatureMode not implemented" << std::endl;
                }
                break;

                case VehicleCommandMode::TrajectorySegmentMode:
                {
                    // TODO
                    std::cerr << "TrajectorySegmentMode not implemented" << std::endl;
                }
                break;
            }


        }
        else {
            spi_mosi_data.LED_bits = 0b10101001;
        }




        // Exchange data with low level controller
        spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));
        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
 

        if(check_CRC_miso(spi_miso_data)) {
            VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
            vehicleState.pose(localization.sensor_update(vehicleState));
            writer_vehicleState.write(vehicleState);
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
