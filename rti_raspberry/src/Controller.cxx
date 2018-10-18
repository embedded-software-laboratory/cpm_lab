#include "Controller.hpp"
#include <iostream>
#include "TrajectoryInterpolation.hpp"


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


double steering_curvature_calibration(double curvature) 
{
    // steady-state curve, from curve fitting
    double steering_servo = (0.241857) * curvature + (-0.035501);   
    return steering_servo;
}


spi_mosi_data_t Controller::get_control_signals(uint64_t stamp_now) {
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

            steering_servo = steering_curvature_calibration(curvature);
            motor_throttle = speed_controller(speed_measured, speed_target);
        }
        break;

        case VehicleCommandMode::TrajectoryMode:
        {
            for(TrajectoryPoint trajectory_point : m_vehicleCommand.data().trajectory_points()) {
                trajectory_points[trajectory_point.t().nanoseconds()] = trajectory_point;
            }            

            //std::cout << "trajectory_point stamp: " << trajectory_point.t().nanoseconds()
            //    << " --- total nodes: " << trajectory_points.size() << std::endl;

            // Find active segment
            auto iterator_segment_end = trajectory_points.lower_bound(stamp_now);
            if(iterator_segment_end != trajectory_points.end()
                && iterator_segment_end != trajectory_points.begin()) 
            {
                auto iterator_segment_start = iterator_segment_end;
                iterator_segment_start--;
                TrajectoryPoint start_point = (*iterator_segment_start).second;
                TrajectoryPoint end_point = (*iterator_segment_end).second;
                assert(stamp_now >= start_point.t().nanoseconds());
                assert(stamp_now <= end_point.t().nanoseconds());

                // We have a valid trajectory segment.
                // Interpolate for the current time.
                TrajectoryInterpolation trajectory_interpolation(stamp_now, start_point, end_point);


                const double x_ref = trajectory_interpolation.position_x;
                const double y_ref = trajectory_interpolation.position_y;
                const double yaw_ref = trajectory_interpolation.yaw;

                const double x = m_vehicleState.pose().x();
                const double y = m_vehicleState.pose().y();
                const double yaw = m_vehicleState.pose().yaw();

                double longitudinal_error =  cos(yaw_ref) * (x-x_ref)  + sin(yaw_ref) * (y-y_ref);
                double lateral_error      = -sin(yaw_ref) * (x-x_ref)  + cos(yaw_ref) * (y-y_ref);
                const double yaw_error = sin(yaw - yaw_ref);

                lateral_error = fmin(0.9,fmax(-0.9, lateral_error));
                longitudinal_error = fmin(0.9,fmax(-0.9, longitudinal_error));


                // Linear lateral controller
                const double curvature = trajectory_interpolation.curvature - 1.0000 * lateral_error - 2.2650 * yaw_error;

                // Linear longitudinal controller
                const double speed_target = trajectory_interpolation.speed - 0.5 * longitudinal_error;

                const double speed_measured = m_vehicleState.speed();
                steering_servo = steering_curvature_calibration(curvature);
                motor_throttle = speed_controller(speed_measured, speed_target);
            }
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


