#pragma once
#include <stdint.h>

void servo_pwm_set_steering(uint32_t signal_microseconds);
void servo_pwm_set_motor(uint32_t signal_microseconds);
void servo_pwm_set_steering_and_motor(uint32_t signal_steering, uint32_t signal_motor);
void init_servo_pwm();