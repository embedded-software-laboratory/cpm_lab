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

double Controller::speed_controller(const double speed_measured, const double speed_target) {

    // steady-state curve, from curve fitting
    double motor_throttle = ((speed_target>=0)?(1.0):(-1.0)) * pow(fabs(0.152744 * speed_target),(0.627910));

    const double speed_error = speed_target - speed_measured;

    // PI controller for the speed
    const double integral_gain = 0.01;
    const double proportional_gain = 0.3;
    speed_throttle_error_integral += integral_gain * speed_error;

    speed_throttle_error_integral = fmin(0.5, fmax(-0.5, speed_throttle_error_integral)); // integral clamping

    motor_throttle += speed_throttle_error_integral;
    motor_throttle += proportional_gain * speed_error;
    return motor_throttle;
}


spi_mosi_data_t Controller::get_control_signals() {
    spi_mosi_data_t spi_mosi_data;
    memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));

    if(emergency_stop) {
        spi_mosi_data.motor_mode = SPI_MOTOR_MODE_BRAKE;
        spi_mosi_data.LED_bits = LED1_BLINK_SLOW | LED2_OFF | LED3_OFF | LED4_OFF;
        return spi_mosi_data;
    }


    double motor_throttle = 0;
    double steering_servo = 0;

    switch(m_vehicleCommand.data()._d().underlying()) {
        
        case VehicleCommandMode::DirectControlMode:
        {
            motor_throttle = fmax(-1.0, fmin(1.0, m_vehicleCommand.data().direct_control().motor_throttle()));
            steering_servo = fmax(-1.0, fmin(1.0, m_vehicleCommand.data().direct_control().steering_servo()));
        }
        break;

        case VehicleCommandMode::SpeedCurvatureMode:
        {
            const double speed_target = m_vehicleCommand.data().speed_curvature().speed();
            const double curvature    = m_vehicleCommand.data().speed_curvature().curvature();
            const double speed_measured = m_vehicleState.speed();

            // steady-state curve, from curve fitting
            steering_servo = (0.241857) * curvature + (-0.035501);
            motor_throttle = speed_controller(speed_measured, speed_target);
        }
        break;

        case VehicleCommandMode::TrajectorySegmentMode:
        {
            // TODO
            std::cerr << "TrajectorySegmentMode not implemented" << std::endl;
        }
        break;
    }


    // Motor deadband, to prevent small stall currents when standing still
    uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
    if(motor_throttle > 0.05) motor_mode = SPI_MOTOR_MODE_FORWARD;
    if(motor_throttle < -0.05) motor_mode = SPI_MOTOR_MODE_REVERSE;


    spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
    spi_mosi_data.servo_command = int16_t(steering_servo * 1000.0);
    spi_mosi_data.motor_mode = motor_mode;
    spi_mosi_data.LED_bits = LED1_OFF | LED2_BLINK_SLOW | LED3_OFF | LED4_OFF;


    return spi_mosi_data;
}


