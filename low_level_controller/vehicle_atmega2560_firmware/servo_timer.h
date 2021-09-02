/**
 * \file servo_timer.h
 *
 * \date 24.09.2018 16:18:47
 * \author: maczijewski
 * 
 * \brief This module provides an interface for setting up and controlling the servo via PWM,
 *        where the servo PWM signal runs at 50 Hz
 */ 


#ifndef SERVO_TIMER_H_
#define SERVO_TIMER_H_


#include <stdint.h>


/**
 * \brief Returns the time since chip startup, in 20 msec increments.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
uint32_t get_tick();

/**
 * \brief Sets up all relevant registers such that the pwm for the servo works properly.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void servo_timer_setup();

/**
 * \brief Sets the position of the servo by providing a PWM value in the range [1800,4200]
 *        where 3000 is the middle value where the vehicle drives straight.
 * \param pwm
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void set_servo_pwm(uint16_t pwm);


#endif /* SERVO_TIMER_H_ */