#include "speed_control.h"
#include "odometer.h"
#include "remote_config.h"
#include "remote_debug.h"



float error_integral = 0;

float speed_control_get_motor_signal(float reference_speed) {

    if(reference_speed == 0) {
        error_integral = 0;

        // during standstill, return the center value 1500, to avoid small stall currents in the motor
        return 1500;
    }

    // open loop speed curve
    float motor_signal_ref = 1450. - 25. * reference_speed;


    // closed loop PI control
    float odom_speed = get_odometer_speed();
    float speed_error = reference_speed - odom_speed;
    error_integral += speed_error;
    if(error_integral > 500) error_integral = 500; // integral clamping
    else if(error_integral < -500) error_integral = -500;

    //printf("refspd %f spderr %f int %f\n", reference_speed, speed_error, error_integral);

    return motor_signal_ref 
        - CONFIG_VAR_speed_error_gain * speed_error
        - CONFIG_VAR_speed_error_integral_gain * error_integral;
}