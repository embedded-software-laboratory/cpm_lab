/**
 * \file motor.h
 * 
 * \brief This module provides an interface for setting up and controlling speed and direction of motor movement.
 *
 * \date 20.09.2018 10:02:02
 * \author maczijewski
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_


#include <stdint.h>

/**
 * \brief Can be used in \link motor_set_direction \endlink to force motor to brake.
 * \ingroup low_level_controller
 */
#define MOTOR_DIRECTION_BRAKE 0

/**
 * \brief Can be used in \link motor_set_direction \endlink to set motor direction to forward.
 * \ingroup low_level_controller
 */
#define MOTOR_DIRECTION_FORWARD 1

/**
 * \brief Can be used in \link motor_set_direction \endlink to set motor direction to backwards.
 * \ingroup low_level_controller
 */
#define MOTOR_DIRECTION_REVERSE 2

/**
 * \brief Set the direction in which the vehicle should move. Three values are possible,
 *        see \link MOTOR_DIRECTION_BRAKE \endlink, \link MOTOR_DIRECTION_FORWARD \endlink,
 *        and \link MOTOR_DIRECTION_REVERSE \endlink.
 * \param direction
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void motor_set_direction(uint8_t direction);

/**
 * \brief Allows to set the duty of the motor. Direction of movement depends on \link motor_set_direction \endlink.
 * \param duty PWM signal within a range of [0,400]
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void motor_set_duty(uint16_t duty);

/**
 * \brief Sets up all relevant registers such that the motor can be controlled afterwards.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void motor_setup();


#endif /* MOTOR_H_ */