#include "Controller.hpp"
#include <iostream>



void Controller::update_vehicle_state(VehicleState vehicleState) {
    m_vehicleState = vehicleState;
}

void Controller::update_command(VehicleCommand vehicleCommand) {
    m_vehicleCommand = vehicleCommand;
    emergency_stop = false;
}


void Controller::vehicle_emergency_stop() {
    emergency_stop = true;
}


spi_mosi_data_t Controller::get_control_signals() {
    spi_mosi_data_t spi_mosi_data;
    memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));


    if(emergency_stop) {
        spi_mosi_data.motor_mode = SPI_MOTOR_MODE_BRAKE;
        spi_mosi_data.LED_bits = LED1_BLINK_SLOW | LED2_OFF | LED3_OFF | LED4_OFF;
    }
    else {

        switch(m_vehicleCommand.data()._d().underlying()) {
            
            case VehicleCommandMode::DirectControlMode:
            {
                double motor_throttle = fmax(-1.0, fmin(1.0, m_vehicleCommand.data().direct_control().motor_throttle()));

                // TODO steering angle calibration / units
                double steering_angle = fmax(-1.0, fmin(1.0, m_vehicleCommand.data().direct_control().steering_angle()));

                uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
                if(motor_throttle > 0.05) motor_mode = SPI_MOTOR_MODE_FORWARD;
                if(motor_throttle < -0.05) motor_mode = SPI_MOTOR_MODE_REVERSE;


                spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
                spi_mosi_data.servo_command = int16_t(steering_angle * 1000.0);
                spi_mosi_data.motor_mode = motor_mode;
                spi_mosi_data.LED_bits = LED1_OFF | LED2_BLINK_SLOW | LED3_OFF | LED4_OFF;
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

    return spi_mosi_data;
}


