#include "speed_control.h"
#include "odometer.h"
#include "remote_config.h"



float error_integral = 0;

float speed_control_get_motor_signal(float reference_speed) {

    if(reference_speed == 0) error_integral = 0;

    // open loop speed curve
    float motor_signal_ref = 1450. - 25. * reference_speed;

    float odometer_speed = get_odometer_speed();

    float speed_error = reference_speed - odometer_speed;

    error_integral += speed_error;
    if(error_integral > 500) error_integral = 500;
    else if(error_integral < -500) error_integral = -500;

    return motor_signal_ref 
        - CONFIG_VAR_speed_error_gain * speed_error
        - CONFIG_VAR_speed_error_integral_gain * error_integral;
}