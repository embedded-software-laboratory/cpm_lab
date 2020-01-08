/*
 * servo_timer.h
 *
 * Created: 24.09.2018 16:18:47
 *  Author: maczijewski
 */ 


#ifndef SERVO_TIMER_H_
#define SERVO_TIMER_H_


#include <stdint.h>


static volatile uint32_t tick_counter = 0;

// The servo PWM signal runs at 50Hz

uint32_t get_tick(); // time since chip startup, in 20 msec increments
void set_servo_pwm(uint16_t pwm); // values from 2000 to 4000. center at 3000


#endif /* SERVO_TIMER_H_ */