#include "Simulation.hpp"
#include <string.h>
#include <math.h>
#include <iostream>

extern "C" {
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h"
}


Simulation::Simulation() 
{
    memset(&input_next, 0, sizeof(spi_mosi_data_t));
    crcInit();
}


spi_miso_data_t Simulation::update(const spi_mosi_data_t spi_mosi_data, const double dt)
{
    // save one input sample to simulate delay time
    spi_mosi_data_t input_now = input_next;
    input_next = spi_mosi_data;

    // init result
    spi_miso_data_t spi_miso_data;
    memset(&spi_miso_data, 0, sizeof(spi_miso_data_t));

    tick++;

    const double T_curvature = 0.09; // steering PT1 time constant in seconds
    const double T_speed = 0.7; // speed PT1 time constant in seconds
    const double curvature_ref = ((input_now.servo_command/(-1000.0)) + 0.035)/(0.24);
    double speed_ref = 6.5 * pow((input_now.motor_pwm/400.0), 1.6);

    if(input_now.motor_mode == SPI_MOTOR_MODE_BRAKE) 
    {
        speed_ref = 0;
    }
    else if(input_now.motor_mode == SPI_MOTOR_MODE_REVERSE) 
    {
        speed_ref = -speed_ref;
    }


    // solve ODE timestep
    const int N_substeps = 5;
    for (int i = 0; i < N_substeps; ++i)
    {
        speed +=     (dt/N_substeps) * ((speed_ref-speed)/T_speed);
        curvature += (dt/N_substeps) * ((curvature_ref-curvature)/T_curvature);
        yaw +=       (dt/N_substeps) * (curvature*speed);
        x +=         (dt/N_substeps) * (speed*cos(yaw));
        y +=         (dt/N_substeps) * (speed*sin(yaw));
        distance +=  (dt/N_substeps) * (speed);
    }


    if(yaw > 0) yaw -= 2*M_PI;
    if(yaw < -2*M_PI) yaw += 2*M_PI;


    spi_miso_data.tick                      = tick;
    spi_miso_data.odometer_steps            = distance/0.003122;
    spi_miso_data.imu_yaw                   = yaw/(-0.00109083078249645598);
    spi_miso_data.imu_acceleration_forward  = ((speed_ref-speed)/T_speed)*100;
    spi_miso_data.imu_acceleration_left     = curvature*speed*speed*100;
    spi_miso_data.speed                     = speed/0.003122/0.2384;
    spi_miso_data.battery_voltage           = (7.2 - 0.2 * fabs(speed_ref-speed))/0.01166;
    spi_miso_data.motor_current             = fabs(((speed_ref-speed) * 0.2)/0.01);
    spi_miso_data.debugC                    = 0;
    spi_miso_data.debugD                    = 0;
    spi_miso_data.status_flags              = 0;



    /*std::cout 
    << "curvature_ref" << "  " << curvature_ref << std::endl
    << "speed_ref" << "  " << speed_ref << std::endl
    << "dt" << "  " << dt << std::endl
    << "speed" << "  " << speed << std::endl
    << "curvature" << "  " << curvature << std::endl
    << "yaw" << "  " << yaw << std::endl
    << "spi_miso_data.imu_yaw" << "  " << spi_miso_data.imu_yaw << std::endl
    << "x" << "  " << x << std::endl
    << "distance" << "  " << distance << std::endl
    << "===============================" << std::endl;*/

    spi_miso_data.CRC = crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));

    return spi_miso_data;
}